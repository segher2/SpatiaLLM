#include "pcg/params.hpp"
#include "pcg/recon/poisson.hpp"
#include "pcg/recon/af.hpp"
#include "pcg/geometry/volume.hpp"

#include <CGAL/IO/polygon_mesh_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>

#include <filesystem>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

namespace fs = std::filesystem;

static bool is_ply(const fs::path& p) { return p.has_extension() && p.extension() == ".ply"; }
static bool has_suffix(const std::string& s, const std::string& suf) {
    return s.size() >= suf.size() && std::equal(suf.rbegin(), suf.rend(), s.rbegin());
}

// Derive object_code and class from cluster filename
// Input stem should be of the form: <object_code>_<class>_cluster
// Returns pair {object_code, class}. If parsing fails, falls back to whole stem and empty class.
static std::pair<std::string, std::string> derive_object_code_and_class(const fs::path& ply_path) {
    std::string stem = ply_path.stem().string();
    constexpr const char* suffix = "_cluster";
    if (stem.size() > strlen(suffix) && has_suffix(stem, suffix)) {
        stem = stem.substr(0, stem.size() - strlen(suffix));
    }
    // split by '_'
    std::vector<std::string> parts;
    {
        std::string cur;
        for (char ch : stem) {
            if (ch == '_') { parts.push_back(cur); cur.clear(); }
            else cur.push_back(ch);
        }
        parts.push_back(cur);
    }
    if (parts.size() >= 2) {
        // last token is class, the rest re-joined by '_' is object_code (though object_code itself shouldn't contain '_')
        std::string obj_code;
        for (size_t i = 0; i + 1 < parts.size(); ++i) {
            if (!obj_code.empty()) obj_code.push_back('_');
            obj_code += parts[i];
        }
        std::string klass = parts.back();
        return {obj_code, klass};
    }
    return {stem, std::string{}};
}

static fs::path find_config_path() {
    const fs::path candidates[] = {
        fs::path("data/configs/default.yaml"),
        fs::path("../data/configs/default.yaml"),
        fs::path("../../data/configs/default.yaml")
    };
    for (const auto& p : candidates) if (fs::exists(p)) return p; return {};
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: pcg_reconstruct <input_root_or_room_dir_or_ply> <output_root_dir> [only_substring]\n";
    std::cerr << "  Input can be: site root (rooms/<site>), floor dir, a single room dir containing results/filtered_clusters,\n";
    std::cerr << "               or a single cluster .ply file under results/filtered_clusters/...\n";
        std::cerr << "  If only_substring is provided, only cluster files whose names contain the substring will be processed.\n";
        return 1;
    }
    fs::path input_root = argv[1];
    fs::path output_root = argv[2];
    std::string only_substr;
    if (argc >= 4) only_substr = argv[3];

    // Reduce PCL console noise: hide non-critical PLYReader warnings (e.g., unhandled 'camera' properties)
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // Load params
    pcg::ParamsConfig cfg; std::string err;
    pcg::load_params_from_file(find_config_path().string(), cfg, &err);
    pcg::recon::PoissonParams pparams{};
    pparams.spacing_neighbors = cfg.poisson_spacing_neighbors;
    pparams.normal_neighbors  = cfg.poisson_normal_neighbors;
    pparams.min_points = 0;                 // no minimum point count gate
    pparams.min_oriented_fraction = cfg.poisson_min_oriented_fraction;
    pparams.require_closed_output = cfg.poisson_require_closed;
    pcg::recon::AFParams aparams{};
    aparams.min_points = cfg.af_min_points;
    aparams.require_closed_output = cfg.af_require_closed;

    auto reconstruct_one = [&](const fs::path& ply_path, const fs::path& out_dir, bool json_flag){
        if (!fs::exists(out_dir)) { std::error_code ec; fs::create_directories(out_dir, ec); }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path.string(), *cloud) != 0) {
            if (json_flag) {
                std::cout << "{\n  \"file\": \"" << ply_path.string() << "\",\n  \"status\": \"read_failed\"\n}\n";
            } else {
                std::cerr << "  ✗ Read failed: " << ply_path << "\n";
            }
            return; }

        pcg::recon::Mesh mesh;
        // Always compute convex hull volume of input for validation
        const double hull_vol = pcg::geom::convex_hull_volume(cloud);
        const double invalid_ratio = cfg.poisson_invalid_ratio_vs_hull; // Poisson volume > ratio * hull => invalid
        // Decide base output file stem: object_code_class_mesh
        auto [object_code, klass] = derive_object_code_and_class(ply_path);
        const std::string base_stem = object_code + std::string{"_"} + klass + std::string{"_mesh"};
        bool reconstructed = false;
        try {
            if (pcg::recon::poisson_reconstruct(cloud, mesh, pparams)) {
                // Validate Poisson by volume vs convex hull
                double mesh_vol = pcg::geom::mesh_signed_volume(mesh);
                bool valid = mesh_vol > 0.0 && (hull_vol <= 0.0 || mesh_vol <= invalid_ratio * hull_vol);
                if (valid) {
                    // Write with method suffix "_possion" per requirement
                    const fs::path out_ply = out_dir / (base_stem + std::string{"_possion.ply"});
                    if (!CGAL::IO::write_polygon_mesh(out_ply.string(), mesh, CGAL::parameters::stream_precision(17))) {
                        if (!json_flag) std::cerr << "  ✗ Write failed: " << out_ply << "\n";
                    } else {
                        if (json_flag) {
                            std::cout << "{\n  \"file\": \"" << ply_path.string() << "\",\n  \"method\": \"poisson\",\n  \"mesh\": \"" << out_ply.string() << "\",\n  \"status\": \"ok\"\n}\n";
                        } else {
                            std::cout << "  ✓ Poisson -> " << out_ply << "\n";
                        }
                        reconstructed = true;
                    }
                } else {
                    // Invalid Poisson by volume check -> force AF fallback
                    std::cout << "  ! Poisson invalid by volume (mesh=" << mesh_vol
                              << ", hull=" << hull_vol << ") -> fallback AF\n";
                }
            }
        } catch (const std::exception& e) {
            if (!json_flag) std::cerr << "  ! Poisson threw exception, will fallback to AF: " << e.what() << "\n";
        } catch (...) {
            if (!json_flag) std::cerr << "  ! Poisson threw unknown exception, will fallback to AF\n";
        }
        if (reconstructed) return; // Poisson accepted; do not run AF

        // Fallback to AF (also guarded)
        try {
            if (pcg::recon::af_reconstruct(cloud, mesh, aparams)) {
                // Write with method suffix "_af"
                const fs::path out_ply = out_dir / (base_stem + std::string{"_af.ply"});
                if (!CGAL::IO::write_polygon_mesh(out_ply.string(), mesh, CGAL::parameters::stream_precision(17))) {
                    if (!json_flag) std::cerr << "  ✗ Write failed: " << out_ply << "\n";
                } else {
                    if (json_flag) {
                        std::cout << "{\n  \"file\": \"" << ply_path.string() << "\",\n  \"method\": \"af\",\n  \"mesh\": \"" << out_ply.string() << "\",\n  \"status\": \"ok\"\n}\n";
                    } else {
                        std::cout << "  ✓ AF -> " << out_ply << "\n";
                    }
                    reconstructed = true;
                }
            }
        } catch (const std::exception& e) {
            if (!json_flag) std::cerr << "  ! AF threw exception: " << e.what() << "\n";
        } catch (...) {
            if (!json_flag) std::cerr << "  ! AF threw unknown exception\n";
        }
        if (!reconstructed) {
            if (json_flag) {
                std::cout << "{\n  \"file\": \"" << ply_path.string() << "\",\n  \"status\": \"failed\"\n}\n";
            } else {
                std::cout << "  ✗ Reconstruction failed: " << ply_path << "\n";
            }
        }
    };

    auto process_room = [&](const fs::path& room_dir){
    const fs::path diag = room_dir / "results" / "filtered_clusters";
        if (!fs::exists(diag)) return;
        // For each object stem folder, traverse *.ply excluding *_uobb.ply
        for (auto& obj_dir : fs::directory_iterator(diag)) {
            if (!obj_dir.is_directory()) continue;
            for (auto& ply_entry : fs::directory_iterator(obj_dir.path())) {
                if (!ply_entry.is_regular_file()) continue;
                if (!is_ply(ply_entry.path())) continue;
                const std::string name = ply_entry.path().filename().string();
                if (has_suffix(name, "_uobb.ply")) continue;
                if (!only_substr.empty() && name.find(only_substr) == std::string::npos) continue;
                // Output next to results as results/recon/<stem>/
                const fs::path out_dir = room_dir / "results" / "recon" / obj_dir.path().filename();
                reconstruct_one(ply_entry.path(), out_dir, cfg.json_output);
            }
        }
    };


    // Single-file (cluster .ply) mode
    if (fs::is_regular_file(input_root) && is_ply(input_root)) {
        // Derive output directory as: <output_root>/results/recon/<obj_stem>/
        fs::path obj_stem = input_root.parent_path().filename(); // e.g., door_007
        fs::path out_dir = output_root / "results" / "recon" / obj_stem;
        reconstruct_one(input_root, out_dir, cfg.json_output);
        return 0;
    }

    // input_root can be site -> floors -> rooms, or directly a room
    if (fs::exists(input_root / "results" / "filtered_clusters")) {
        process_room(input_root);
        return 0;
    }

    auto has_room_signature = [](const fs::path& p){ return fs::exists(p / "results" / "filtered_clusters"); };

    // Iterate floors → rooms
    for (auto& floor_entry : fs::directory_iterator(input_root)) {
        if (!floor_entry.is_directory()) continue;
        for (auto& room_entry : fs::directory_iterator(floor_entry.path())) {
            if (!room_entry.is_directory()) continue;
            if (!has_room_signature(room_entry.path())) continue;
            process_room(room_entry.path());
        }
    }
    return 0;
}
