// Clean single copy of the implementation (removed duplicates and BOM)
#include "pcg/ply_io.hpp"
#include "pcg/clustering.hpp"
#include "pcg/params.hpp"
#include "pcg/bbox.hpp"
#include "pcg/csv_writer.hpp"

#include <filesystem>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>
#include <limits>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

namespace fs = std::filesystem;

static bool is_ply(const fs::path& p) { return p.has_extension() && p.extension() == ".ply"; }

static bool has_any_ply(const fs::path& dir) {
    std::error_code ec;
    if (!fs::exists(dir, ec) || !fs::is_directory(dir, ec)) return false;
    for (auto& e : fs::directory_iterator(dir)) {
        if (e.is_regular_file() && is_ply(e.path())) return true;
    }
    return false;
}

static bool name_contains_shell(const std::string& name) {
    std::string s = name;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s.find("shell") != std::string::npos;
}

// trim helpers for parsing CSV headers
static inline void ltrim_inplace(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
}
static inline void rtrim_inplace(std::string& s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
}
static inline void trim_inplace(std::string& s) { ltrim_inplace(s); rtrim_inplace(s); }

// Attempt to read room_id and floor_id from object_manifest.csv under a room directory.
// Returns true on success; otherwise false.
static bool read_room_and_floor_from_manifest(const fs::path& room_dir, int& out_room_id, int& out_floor_id) {
    const fs::path manifest = room_dir / "object_manifest.csv";
    std::ifstream fin(manifest);
    if (!fin) return false;
    std::string header;
    if (!std::getline(fin, header)) return false;
    // tokenize header by ','
    std::vector<std::string> cols;
    std::string token;
    for (char ch : header) {
        if (ch == ',') {
            trim_inplace(token);
            cols.push_back(token);
            token.clear();
        } else {
            token.push_back(ch);
        }
    }
    trim_inplace(token);
    if (!token.empty() || (!header.empty() && header.back() == ',')) cols.push_back(token);
    int idx_room = -1, idx_floor = -1;
    for (size_t i = 0; i < cols.size(); ++i) {
        std::string c = cols[i];
        // normalize to lower and remove spaces
        c.erase(std::remove_if(c.begin(), c.end(), [](unsigned char ch){ return std::isspace(ch); }), c.end());
        std::transform(c.begin(), c.end(), c.begin(), [](unsigned char ch){ return static_cast<char>(std::tolower(ch)); });
        if (c == "room_id") idx_room = static_cast<int>(i);
        if (c == "floor_id") idx_floor = static_cast<int>(i);
    }
    if (idx_room < 0 || idx_floor < 0) return false;
    // read first data row (non-empty)
    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        // split
        std::vector<std::string> vals;
        std::string t;
        for (char ch : line) {
            if (ch == ',') {
                trim_inplace(t);
                vals.push_back(t);
                t.clear();
            } else {
                t.push_back(ch);
            }
        }
        trim_inplace(t);
        if (!t.empty() || (!line.empty() && line.back() == ',')) vals.push_back(t);
        if (static_cast<int>(vals.size()) <= std::max(idx_room, idx_floor)) continue;
        try {
            out_room_id = std::stoi(vals[static_cast<size_t>(idx_room)]);
            out_floor_id = std::stoi(vals[static_cast<size_t>(idx_floor)]);
            return true;
        } catch (...) {
            return false;
        }
    }
    return false;
}

// Fallback: parse integers from folder names like floor_1 and room_001
static void derive_room_and_floor_from_paths(const fs::path& room_dir, int& out_room_id, int& out_floor_id) {
    out_room_id = -1; out_floor_id = -1;
    const std::string room_name = room_dir.filename().string();
    const std::string floor_name = room_dir.parent_path().filename().string();
    auto extract_int = [](const std::string& s) -> int {
        std::string digits;
        for (char ch : s) if (std::isdigit(static_cast<unsigned char>(ch))) digits.push_back(ch);
        if (digits.empty()) return -1; return std::stoi(digits);
    };
    out_room_id = extract_int(room_name);
    out_floor_id = extract_int(floor_name);
}

static fs::path find_config_path_from_cwd_or_repo() {
    // Try common locations relative to current working dir
    fs::path candidates[] = {
        fs::path("data/configs/default.yaml"),
        fs::path("../data/configs/default.yaml"),
        fs::path("../../data/configs/default.yaml")
    };
    for (const auto& p : candidates) {
        if (fs::exists(p)) return p;
    }
    return fs::path();
}

static void process_one_room(const fs::path& room_in,
                             const fs::path& room_out,
                             const pcg::ParamsConfig& cfg,
                             double radius,
                             int min_cluster,
                             int max_neighbors)
{
    std::error_code ec;
    fs::create_directories(room_out, ec);
    fs::path diag_dir = room_out / "results";
    fs::create_directories(diag_dir, ec);

    std::ofstream log(diag_dir / (room_in.filename().string() + std::string{".log"}));
    const std::string csv_path = (room_out / (room_in.filename().string() + std::string{".csv"})).string();
    pcg::CsvWriter csv(csv_path);
    if (csv.good()) {
        csv.writeHeader({
            "object_code",
            "object_id","room_id","floor_id",
            "class",
            "file","cluster_id",
            "center_x","center_y","center_z",
            "size_x","size_y","size_z",
            "yaw_rad"
        });
    } else {
        if (log) log << "WARNING: cannot open CSV: " << csv_path << "\n";
    }
    if (!log) {
        std::cerr << "Failed to open results log for writing: " << diag_dir << "\n";
    }

    // Determine identifiers from manifest or fallback to folder names
    int manifest_room_id = -1, manifest_floor_id = -1;
    if (!read_room_and_floor_from_manifest(room_in, manifest_room_id, manifest_floor_id)) {
        derive_room_and_floor_from_paths(room_in, manifest_room_id, manifest_floor_id);
    }
    std::size_t running_object_id = 0; // sequential object_id starting at 0 per room

    size_t files_processed = 0;
    for (auto& entry : fs::directory_iterator(room_in)) {
        if (!entry.is_regular_file()) continue;
        if (!is_ply(entry.path())) continue;

        std::string err;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb;
        bool has_color = false;
        if (!pcg::load_ply_xyz_or_rgb(entry.path().string(), cloud, cloud_rgb, has_color, &err)) {
            std::cerr << "[ERROR] Load PLY failed: " << entry.path() << " => " << err << "\n";
            if (log) log << "ERROR: " << entry.path().string() << " => " << err << "\n";
            continue;
        }

        const std::string stem = entry.path().stem().string();
        // Derive class from filename stem: take prefix before first '_', lowercased
        std::string klass;
        {
            std::string tmp = stem;
            auto pos = tmp.find('_');
            klass = (pos == std::string::npos) ? tmp : tmp.substr(0, pos);
            std::transform(klass.begin(), klass.end(), klass.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        }
        std::vector<pcl::PointIndices> clusters;
        std::vector<pcl::PointIndices> filtered;
        bool is_shell = name_contains_shell(stem);

        // New rule: filenames containing "shell" are treated as whole cloud (no clustering/filtering)
        if (is_shell) {
            pcl::PointIndices one;
            one.indices.reserve(cloud->size());
            for (std::size_t i = 0; i < cloud->size(); ++i) one.indices.push_back(static_cast<int>(i));
            clusters.push_back(std::move(one));
            filtered = clusters;
            if (log) {
                log << "  Detected *shell* -> treat as single cluster, skip clustering and filtering\n";
            }
        } else {
            pcg::ClusteringParams params;
            params.radius = radius;
            params.min_cluster_size = min_cluster;
            params.max_neighbors = max_neighbors;
            clusters = pcg::cluster_cloud(cloud, params);

            if (!clusters.empty()) {
                std::size_t min_sz = clusters[0].indices.size();
                std::size_t max_sz = clusters[0].indices.size();
                std::size_t total_pts = 0;
                for (const auto& c : clusters) {
                    const std::size_t s = c.indices.size();
                    total_pts += s;
                    if (s < min_sz) min_sz = s;
                    if (s > max_sz) max_sz = s;
                }
                const double ratio = (min_sz == 0) ? std::numeric_limits<double>::infinity()
                                                   : static_cast<double>(max_sz) / static_cast<double>(min_sz);
                const bool skip_filter = std::isfinite(ratio) && (ratio <= cfg.no_filter_ratio);
                if (skip_filter) {
                    filtered = clusters;
                } else {
                    filtered = pcg::filter_clusters_by_average(clusters, cfg.filter_factor);
                }
                if (log) {
                    log << "  Cluster size stats: min=" << min_sz << ", max=" << max_sz
                        << ", ratio=" << (min_sz == 0 ? 0.0 : static_cast<double>(max_sz)/static_cast<double>(min_sz))
                        << ", no_filter_ratio=" << cfg.no_filter_ratio
                        << (skip_filter ? " -> skip filtering" : " -> apply filtering")
                        << "\n";
                }
            }
        }

        if (is_shell) {
            std::cout << "File: " << entry.path().filename().string()
                      << ", points: " << cloud->size()
                      << ", shell -> skip clustering/filtering\n";
            if (log) {
                log << "File: " << entry.path().filename().string()
                    << ", points: " << cloud->size()
                    << ", shell -> skip clustering/filtering\n";
            }
        } else {
            std::cout << "File: " << entry.path().filename().string()
                      << ", points: " << cloud->size()
                      << ", clusters(raw/filtered): " << clusters.size() << "/" << filtered.size() << "\n";
            if (log) {
                log << "File: " << entry.path().filename().string()
                    << ", points: " << cloud->size()
                    << ", clusters(raw/filtered): " << clusters.size() << "/" << filtered.size() << "\n";
                std::size_t total_clustered_points = 0;
                for (const auto& c : clusters) total_clustered_points += c.indices.size();
                const double avg = clusters.empty() ? 0.0 : static_cast<double>(total_clustered_points) / clusters.size();
                const double thr = avg * cfg.filter_factor;
                (void)thr;
                log << "  Avg cluster size: " << avg << ", filter threshold (factor=" << cfg.filter_factor << "): " << thr << "\n";
                for (size_t ci = 0; ci < clusters.size(); ++ci) {
                    log << "  - raw cluster " << ci << ": size=" << clusters[ci].indices.size() << "\n";
                }
                for (size_t ci = 0; ci < filtered.size(); ++ci) {
                    log << "  - kept cluster " << ci << ": size=" << filtered[ci].indices.size() << "\n";
                }
            }
        }

        try {
            fs::path clusters_dir = is_shell ? (diag_dir / "shell" / stem)
                                             : (diag_dir / "filtered_clusters" / stem);
            fs::create_directories(clusters_dir, ec);
            std::size_t saved = 0;
            for (std::size_t ci = 0; ci < filtered.size(); ++ci) {
                const auto& inds = filtered[ci].indices;
                // Always build XYZ subcloud for geometry and UOBB
                pcl::PointCloud<pcl::PointXYZ>::Ptr c_xyz(new pcl::PointCloud<pcl::PointXYZ>);
                c_xyz->points.reserve(inds.size());
                for (int idx : inds) c_xyz->points.push_back(cloud->points[static_cast<std::size_t>(idx)]);
                c_xyz->width = static_cast<uint32_t>(c_xyz->points.size());
                c_xyz->height = 1; c_xyz->is_dense = true;

                // Optional RGB subcloud for colored export
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr c_rgb;
                if (!is_shell && has_color && cloud_rgb) {
                    c_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
                    c_rgb->points.reserve(inds.size());
                    for (int idx : inds) c_rgb->points.push_back(cloud_rgb->points[static_cast<std::size_t>(idx)]);
                    c_rgb->width = static_cast<uint32_t>(c_rgb->points.size());
                    c_rgb->height = 1; c_rgb->is_dense = true;
                }

                std::vector<int> contig(c_xyz->size());
                for (std::size_t k=0;k<c_xyz->size();++k) contig[k]=static_cast<int>(k);
                pcg::UOBB box = pcg::compute_uobb(c_xyz, contig);

                // Build object_code using current running_object_id (do not increment yet)
                const std::size_t oid = running_object_id;
                const std::string object_code = std::to_string(manifest_floor_id) + "-" +
                                                std::to_string(manifest_room_id) + "-" +
                                                std::to_string(oid);

                fs::path uobb_name = clusters_dir / (object_code + "_" + klass + "_uobb.ply");
                if (pcg::save_uobb_ply(uobb_name.string(), box)) {
                    ++saved;
                    if (log) log << "  ✓ saved UOBB: " << uobb_name.string() << "\n";
                } else {
                    if (log) log << "  ✗ failed to save UOBB: " << uobb_name.string() << "\n";
                }

                // For shell files, also place the original input shell PLY next to the UOBB
                if (is_shell) {
                    const fs::path shell_copy_name = clusters_dir / (object_code + std::string{"_shell.ply"});
                    std::error_code copy_ec;
                    fs::create_directories(clusters_dir, copy_ec);
                    copy_ec.clear();
                    fs::copy_file(entry.path(), shell_copy_name, fs::copy_options::overwrite_existing, copy_ec);
                    if (copy_ec) {
                        if (log) log << "  ✗ failed to copy shell PLY to: " << shell_copy_name.string() << ", error: " << copy_ec.message() << "\n";
                    } else {
                        if (log) log << "  ✓ copied shell PLY: " << shell_copy_name.string() << "\n";
                    }
                }

                // Also export each non-shell filtered cluster's point cloud as PLY for later surface reconstruction
                if (!is_shell) {
                    fs::path cply_name = clusters_dir / (object_code + "_" + klass + "_cluster.ply");
                    pcl::PLYWriter writer;
                    int rc = -1;
                    if (has_color && c_rgb) rc = writer.write(cply_name.string(), *c_rgb, true);
                    else rc = writer.write(cply_name.string(), *c_xyz, true);
                    if (rc == 0) {
                        if (log) log << "  ✓ saved cluster PLY: " << cply_name.string() << "\n";
                    } else {
                        if (log) log << "  ✗ failed to save cluster PLY: " << cply_name.string() << "\n";
                    }
                }

                if (csv.good()) {
                    csv.writeRow({
                        object_code,
                        std::to_string(oid),
                        std::to_string(manifest_room_id),
                        std::to_string(manifest_floor_id),
                        klass,
                        entry.path().filename().string(), std::to_string(ci),
                        std::to_string(box.center.x()), std::to_string(box.center.y()), std::to_string(box.center.z()),
                        std::to_string(box.size.x()), std::to_string(box.size.y()), std::to_string(box.size.z()),
                        std::to_string(box.yaw)
                    });
                }
                // Now increment object_id for next object
                ++running_object_id;
            }
            if (log) {
                if (is_shell) {
                    log << "Saved " << saved << "/" << filtered.size() << " shell UOBB to: " << (diag_dir / "shell" / stem).string() << "\n";
                } else {
                    log << "Saved " << saved << "/" << filtered.size() << " filtered clusters to: " << (diag_dir / "filtered_clusters" / stem).string() << "\n";
                }
            }
        } catch (const std::exception& e) {
            if (log) log << "  Warning: failed exporting clusters: " << e.what() << "\n";
        }
        ++files_processed;
    }

    if (files_processed == 0) {
        std::cerr << "No .ply files found under: " << room_in << "\n";
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: pcg_room <input_root_dir> [radius] [min_cluster_size]\n";
        std::cerr << "Output will be automatically placed in ./output (cleared before processing)\n";
        return 1;
    }

    fs::path input_root = argv[1];
    fs::path output_root = "output"; // Fixed output directory
    
    // Clear output directory before processing
    std::error_code clear_ec;
    if (fs::exists(output_root, clear_ec)) {
        std::cout << "Clearing existing output directory: " << output_root << "\n";
        fs::remove_all(output_root, clear_ec);
        if (clear_ec) {
            std::cerr << "Warning: Failed to clear output directory: " << clear_ec.message() << "\n";
        }
    }

    // Load params from config file if present
    pcg::ParamsConfig cfg; // defaults
    std::string cfgErr;
    fs::path cfg_path = find_config_path_from_cwd_or_repo();
    if (cfg_path.empty()) {
        // last resort: try relative to output root's ancestors
        fs::path p = output_root;
        for (int i=0;i<4 && !fs::exists(cfg_path);++i) {
            cfg_path = p / "data/configs/default.yaml";
            p = p.parent_path();
        }
    }
    pcg::load_params_from_file(cfg_path.string(), cfg, &cfgErr);

    // CLI overrides (note: argv indices shifted by 1 since output_root is now fixed)
    double radius = (argc >= 3) ? std::stod(argv[2]) : cfg.radius;
    int min_cluster = (argc >= 4) ? std::stoi(argv[3]) : cfg.min_cluster_size;
    const int max_neighbors = cfg.max_neighbors;

    std::error_code ec;
    fs::create_directories(output_root, ec);

    // Process input_root containing floors -> rooms
    size_t rooms_processed = 0;
    for (auto& floor_entry : fs::directory_iterator(input_root)) {
        if (!floor_entry.is_directory()) continue;
        const fs::path floor_dir = floor_entry.path();
        
        // Copy rooms_manifest.csv from floor directory if it exists
        const fs::path manifest_src = floor_dir / "rooms_manifest.csv";
        if (fs::exists(manifest_src)) {
            const fs::path manifest_dst = output_root / floor_dir.filename() / "rooms_manifest.csv";
            std::error_code copy_ec;
            fs::create_directories(manifest_dst.parent_path(), copy_ec);
            copy_ec.clear();
            fs::copy_file(manifest_src, manifest_dst, fs::copy_options::overwrite_existing, copy_ec);
            if (copy_ec) {
                std::cerr << "Warning: Failed to copy " << manifest_src << " to " << manifest_dst 
                          << ": " << copy_ec.message() << "\n";
            } else {
                std::cout << "Copied manifest: " << manifest_src.filename() << " -> " 
                          << manifest_dst.parent_path() << "\n";
            }
        }
        
        for (auto& room_entry : fs::directory_iterator(floor_dir)) {
            if (!room_entry.is_directory()) continue;
            const fs::path room_dir = room_entry.path();
            if (!has_any_ply(room_dir)) continue;
            const fs::path room_out = output_root / floor_dir.filename() / room_dir.filename();
            process_one_room(room_dir, room_out, cfg, radius, min_cluster, max_neighbors);
            ++rooms_processed;
        }
    }

    if (rooms_processed == 0) {
        std::cerr << "No rooms with .ply files found under: " << input_root << "\n";
        return 2;
    }
    return 0;
}
