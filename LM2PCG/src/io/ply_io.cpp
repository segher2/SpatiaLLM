#include "pcg/ply_io.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <cctype>

namespace pcg {

namespace {
struct VertexProperty {
    std::string type;  // e.g., float, double, uchar, etc.
    std::string name;  // e.g., x, y, z, red, green, blue, scalar_label
    bool is_list = false; // for completeness; normally false for vertex
    std::string list_count_type;
    std::string list_elem_type;
};

struct PlyHeader {
    bool binary_little_endian = false;
    uint32_t vertex_count = 0;
    std::vector<VertexProperty> vertex_props;
};

static inline void trim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch){return !std::isspace(ch);}));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch){return !std::isspace(ch);}).base(), s.end());
}

bool parse_header(std::istream& is, PlyHeader& hdr, std::string& err) {
    std::string line;
    if (!std::getline(is, line)) { err = "Empty file"; return false; }
    trim(line);
    if (line != "ply") { err = "Not a PLY file"; return false; }

    bool in_vertex = false;

    while (std::getline(is, line)) {
        trim(line);
        if (line.empty()) continue;
        if (line == "end_header") break;
        std::istringstream ls(line);
        std::string tok;
        if (!(ls >> tok)) continue;
        if (tok == "format") {
            std::string fmt; double ver{0.0};
            ls >> fmt >> ver;
            if (fmt == "binary_little_endian" && (ver == 1.0 || ver == 1)) {
                hdr.binary_little_endian = true;
            } else {
                err = "Unsupported PLY format (need binary_little_endian 1.0)";
                return false;
            }
        } else if (tok == "element") {
            std::string name; uint32_t count{0};
            ls >> name >> count;
            if (name == "vertex") { hdr.vertex_count = count; in_vertex = true; }
            else { in_vertex = false; }
        } else if (tok == "property") {
            if (!in_vertex) continue; // we only care about vertex properties
            std::string maybe_list;
            ls >> maybe_list;
            if (maybe_list == "list") {
                // property list count_type elem_type name
                std::string count_t, elem_t, name;
                ls >> count_t >> elem_t >> name;
                VertexProperty vp; vp.is_list = true; vp.list_count_type = count_t; vp.list_elem_type = elem_t; vp.name = name;
                hdr.vertex_props.push_back(vp);
            } else {
                // property type name
                std::string name = {};
                ls >> name;
                VertexProperty vp; vp.type = maybe_list; vp.name = name;
                hdr.vertex_props.push_back(vp);
            }
        } else if (tok == "comment" || tok == "obj_info") {
            // ignore
        }
    }

    if (!hdr.binary_little_endian) {
        err = "Failed to parse PLY header or wrong endian";
        return false;
    }
    if (hdr.vertex_count == 0) {
        err = "PLY has zero vertices or missing vertex element";
        return false;
    }
    return true;
}

// Helper to read little-endian types regardless of host
template <typename T>
bool read_le(std::istream& is, T& out) {
    char buf[sizeof(T)];
    if (!is.read(buf, sizeof(T))) return false;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    std::memcpy(&out, buf, sizeof(T));
#else
    // swap bytes
    T tmp{};
    for (size_t i = 0; i < sizeof(T); ++i) {
        reinterpret_cast<unsigned char*>(&tmp)[i] = reinterpret_cast<unsigned char*>(buf)[sizeof(T)-1-i];
    }
    out = tmp;
#endif
    return true;
}

static inline std::size_t scalar_type_size(const std::string& t) {
    if (t == "char" || t == "int8" || t == "schar") return 1;
    if (t == "uchar" || t == "uint8") return 1;
    if (t == "short" || t == "int16") return 2;
    if (t == "ushort" || t == "uint16") return 2;
    if (t == "int" || t == "int32") return 4;
    if (t == "uint" || t == "uint32") return 4;
    if (t == "float" || t == "float32") return 4;
    if (t == "double" || t == "float64") return 8;
    return 0; // unknown
}

template <typename T>
bool skip_n(std::istream& is, std::size_t n) {
    if (n == 0) return true;
    std::vector<char> buf(n);
    return static_cast<bool>(is.read(buf.data(), n));
}

} // namespace

bool load_ply_xyz(const std::string& filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                  std::string* error) {
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs) { if (error) *error = "Cannot open file: " + filepath; return false; }

    PlyHeader hdr;
    std::string err;
    if (!parse_header(ifs, hdr, err)) {
        if (error) *error = err; return false;
    }

    cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_out->points.resize(hdr.vertex_count);

    // Precheck: locate x,y,z indices if present
    bool has_x=false, has_y=false, has_z=false;
    for (const auto& p : hdr.vertex_props) {
        if (!p.is_list) {
            if (p.name == "x") has_x = true;
            else if (p.name == "y") has_y = true;
            else if (p.name == "z") has_z = true;
        }
    }
    if (!(has_x && has_y && has_z)) {
        if (error) *error = "PLY vertex properties missing x/y/z";
        return false;
    }

    for (uint32_t i = 0; i < hdr.vertex_count; ++i) {
        double x=0.0, y=0.0, z=0.0; // accumulate as double then cast to float
        for (const auto& prop : hdr.vertex_props) {
            if (prop.is_list) {
                // Unusual for vertex; attempt to skip
                std::size_t count_size = scalar_type_size(prop.list_count_type);
                if (count_size == 0) { if (error) *error = "Unknown list count type in vertex"; return false; }
                uint64_t count = 0;
                if (count_size == 1) { uint8_t v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading list count"; return false; } count = v; }
                else if (count_size == 2) { uint16_t v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading list count"; return false; } count = v; }
                else if (count_size == 4) { uint32_t v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading list count"; return false; } count = v; }
                else if (count_size == 8) { uint64_t v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading list count"; return false; } count = v; }
                std::size_t elem_size = scalar_type_size(prop.list_elem_type);
                if (elem_size == 0) { if (error) *error = "Unknown list elem type in vertex"; return false; }
                if (!skip_n<char>(ifs, elem_size * count)) { if (error) *error = "EOF skipping list elems"; return false; }
                continue;
            }

            std::size_t sz = scalar_type_size(prop.type);
            if (sz == 0) { if (error) *error = "Unknown scalar type in vertex"; return false; }

            if (prop.name == "x" || prop.name == "y" || prop.name == "z") {
                if (prop.type == "float" || prop.type == "float32") {
                    float v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading xyz"; return false; }
                    if (prop.name == "x") x = v; else if (prop.name == "y") y = v; else z = v;
                } else if (prop.type == "double" || prop.type == "float64") {
                    double v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading xyz"; return false; }
                    if (prop.name == "x") x = v; else if (prop.name == "y") y = v; else z = v;
                } else {
                    // xyz must be floating; attempt to read as float-sized fallback
                    float v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading xyz (non-float type)"; return false; }
                    if (prop.name == "x") x = v; else if (prop.name == "y") y = v; else z = v;
                }
            } else {
                // skip other properties
                if (!skip_n<char>(ifs, sz)) { if (error) *error = "EOF skipping property"; return false; }
            }
        }
        cloud_out->points[i].x = static_cast<float>(x);
        cloud_out->points[i].y = static_cast<float>(y);
        cloud_out->points[i].z = static_cast<float>(z);
    }

    cloud_out->width = hdr.vertex_count;
    cloud_out->height = 1;
    cloud_out->is_dense = true;
    return true;
}

bool load_ply_xyz_or_rgb(const std::string& filepath,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz_out,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_rgb_out,
                         bool& has_color,
                         std::string* error) {
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs) { if (error) *error = "Cannot open file: " + filepath; return false; }

    PlyHeader hdr;
    std::string err;
    if (!parse_header(ifs, hdr, err)) {
        if (error) *error = err; return false;
    }

    // determine presence of xyz and rgb
    int x_idx=-1,y_idx=-1,z_idx=-1,r_idx=-1,g_idx=-1,b_idx=-1;
    for (int i=0;i<static_cast<int>(hdr.vertex_props.size());++i) {
        const auto& p = hdr.vertex_props[i];
        if (p.is_list) continue;
        if (p.name == "x") x_idx = i;
        else if (p.name == "y") y_idx = i;
        else if (p.name == "z") z_idx = i;
        else if (p.name == "red") r_idx = i;
        else if (p.name == "green") g_idx = i;
        else if (p.name == "blue") b_idx = i;
    }
    if (x_idx<0 || y_idx<0 || z_idx<0) {
        if (error) *error = "PLY vertex properties missing x/y/z";
        return false;
    }
    has_color = (r_idx>=0 && g_idx>=0 && b_idx>=0);

    // Prepare outputs
    cloud_xyz_out.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_xyz_out->points.resize(hdr.vertex_count);
    if (has_color) {
        cloud_rgb_out.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloud_rgb_out->points.resize(hdr.vertex_count);
    } else {
        cloud_rgb_out.reset();
    }

    // Read interleaved by property order in header
    for (uint32_t i = 0; i < hdr.vertex_count; ++i) {
        double x=0,y=0,z=0; uint8_t R=0,G=0,B=0;
        for (const auto& prop : hdr.vertex_props) {
            if (prop.is_list) {
                std::size_t count_size = scalar_type_size(prop.list_count_type);
                if (count_size == 0) { if (error) *error = "Unknown list count type in vertex"; return false; }
                uint64_t count = 0;
                if (count_size == 1) { uint8_t v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading list count"; return false; } count = v; }
                else if (count_size == 2) { uint16_t v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading list count"; return false; } count = v; }
                else if (count_size == 4) { uint32_t v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading list count"; return false; } count = v; }
                else if (count_size == 8) { uint64_t v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading list count"; return false; } count = v; }
                std::size_t elem_size = scalar_type_size(prop.list_elem_type);
                if (elem_size == 0) { if (error) *error = "Unknown list elem type in vertex"; return false; }
                if (!skip_n<char>(ifs, elem_size * count)) { if (error) *error = "EOF skipping list elems"; return false; }
                continue;
            }

            const std::string& name = prop.name;
            const std::string& type = prop.type;
            std::size_t sz = scalar_type_size(type);
            if (sz == 0) { if (error) *error = "Unknown scalar type in vertex"; return false; }

            if (name == "x" || name == "y" || name == "z") {
                if (type == "float" || type == "float32") { float v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading xyz"; return false; } if (name=="x") x=v; else if (name=="y") y=v; else z=v; }
                else if (type == "double" || type == "float64") { double v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading xyz"; return false; } if (name=="x") x=v; else if (name=="y") y=v; else z=v; }
                else { float v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading xyz (non-float type)"; return false; } if (name=="x") x=v; else if (name=="y") y=v; else z=v; }
            } else if (has_color && (name == "red" || name == "green" || name == "blue")) {
                // accept uchar/uint8; for other sizes, read and cast
                if (type == "uchar" || type == "uint8") {
                    uint8_t v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading color"; return false; }
                    if (name=="red") R=v; else if (name=="green") G=v; else B=v;
                } else if (type == "float" || type == "float32") {
                    float v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading color"; return false; }
                    uint8_t u = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
                    if (name=="red") R=u; else if (name=="green") G=u; else B=u;
                } else if (type == "double" || type == "float64") {
                    double v; if (!read_le(ifs, v)) { if (error) *error = "EOF reading color"; return false; }
                    uint8_t u = static_cast<uint8_t>(std::clamp(v, 0.0, 255.0));
                    if (name=="red") R=u; else if (name=="green") G=u; else B=u;
                } else {
                    // generic skip with reading value then casting best-effort
                    if (!skip_n<char>(ifs, sz)) { if (error) *error = "EOF skipping unsupported color type"; return false; }
                }
            } else {
                if (!skip_n<char>(ifs, sz)) { if (error) *error = "EOF skipping property"; return false; }
            }
        }
        // store
        cloud_xyz_out->points[i].x = static_cast<float>(x);
        cloud_xyz_out->points[i].y = static_cast<float>(y);
        cloud_xyz_out->points[i].z = static_cast<float>(z);
        if (has_color) {
            auto& pt = cloud_rgb_out->points[i];
            pt.x = static_cast<float>(x);
            pt.y = static_cast<float>(y);
            pt.z = static_cast<float>(z);
            pt.r = R; pt.g = G; pt.b = B;
        }
    }

    cloud_xyz_out->width = hdr.vertex_count; cloud_xyz_out->height = 1; cloud_xyz_out->is_dense = true;
    if (has_color) { cloud_rgb_out->width = hdr.vertex_count; cloud_rgb_out->height = 1; cloud_rgb_out->is_dense = true; }
    return true;
}

} // namespace pcg
