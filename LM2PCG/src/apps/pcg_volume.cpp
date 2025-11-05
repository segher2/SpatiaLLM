#include "pcg/geometry/volume.hpp"
#include "pcg/params.hpp"

#include <CGAL/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <iostream>
#include <filesystem>
#include <iomanip>

namespace fs = std::filesystem;

static std::filesystem::path find_config_path() {
    const std::filesystem::path candidates[] = {
        std::filesystem::path("data/configs/default.yaml"),
        std::filesystem::path("../data/configs/default.yaml"),
        std::filesystem::path("../../data/configs/default.yaml")
    };
    for (const auto& p : candidates) if (std::filesystem::exists(p)) return p; return {};
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: pcg_volume [--voxel] <mesh_file_1> [mesh_file_2 ...]\n";
        std::cerr << "  Supported formats (by CGAL): OFF/OBJ/PLY/STL, etc.\n";
        std::cerr << "  --voxel: Force adaptive voxel method (default: signed for closed, voxel for non-closed)\n";
        return 1;
    }

    std::cout.setf(std::ios::fixed); std::cout << std::setprecision(10);
    std::cerr.setf(std::ios::fixed); std::cerr << std::setprecision(10);

    pcg::ParamsConfig cfg; std::string err;
    pcg::load_params_from_file(find_config_path().string(), cfg, &err);
    const bool as_json = cfg.json_output;

    // Check for --voxel flag
    bool use_voxel = false;
    int file_start = 1;
    if (argc > 1 && std::string(argv[1]) == "--voxel") {
        use_voxel = true;
        file_start = 2;
    }

    for (int i = file_start; i < argc; ++i) {
        const fs::path path = argv[i];
        if (!fs::exists(path)) {
            std::cerr << "✗ Not found: " << path << "\n";
            continue;
        }
        pcg::geom::Mesh mesh;
        if (!CGAL::IO::read_polygon_mesh(path.string(), mesh) || mesh.is_empty()) {
            std::cerr << "✗ Read failed or empty: " << path << "\n";
            continue;
        }

        // Triangulate all polygon faces to ensure correct volume calculation
        CGAL::Polygon_mesh_processing::triangulate_faces(mesh);

        bool is_closed = CGAL::is_closed(mesh);
        double vol = 0.0;
        double vol_voxel = 0.0;
        bool computed = true;
        std::string method = "";

        if (use_voxel) {
            // Use adaptive voxel method (works for both closed and non-closed)
            vol_voxel = pcg::geom::mesh_volume_adaptive_voxel(mesh);
            method = "adaptive_voxel";
        } else if (is_closed) {
            // Use signed volume method (only for closed meshes)
            vol = pcg::geom::mesh_signed_volume(mesh);
            method = "signed_volume";
        } else {
            // For non-closed meshes, automatically use adaptive voxel method
            vol_voxel = pcg::geom::mesh_volume_adaptive_voxel(mesh);
            method = "adaptive_voxel";
        }

        if (as_json) {
            std::cout << "{\n";
            std::cout << "  \"file\": \"" << path.string() << "\",\n";
            std::cout << "  \"closed\": " << (is_closed ? "true" : "false") << ",\n";
            std::cout << "  \"method\": \"" << method << "\",\n";
            std::cout << "  \"volume\": " << (method == "signed_volume" ? vol : vol_voxel) << "\n";
            std::cout << "}\n";
        } else {
            std::cout << path << "\n"
                      << "  closed: " << (is_closed ? "true" : "false") << "\n"
                      << "  method: " << method << "\n"
                      << "  volume: " << (method == "signed_volume" ? vol : vol_voxel) << "\n";
        }
    }
    return 0;
}
