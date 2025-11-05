#include "pcg/geometry/area.hpp"
#include "pcg/params.hpp"

#include <CGAL/IO/polygon_mesh_io.h>
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
        std::cerr << "Usage: pcg_area <mesh_file_1> [mesh_file_2 ...]\n";
        std::cerr << "  Supported formats (by CGAL): OFF/OBJ/PLY/STL, etc.\n";
        return 1;
    }

    std::cout.setf(std::ios::fixed); std::cout << std::setprecision(10);
    std::cerr.setf(std::ios::fixed); std::cerr << std::setprecision(10);

    pcg::ParamsConfig cfg; std::string err;
    pcg::load_params_from_file(find_config_path().string(), cfg, &err);
    const bool as_json = cfg.json_output;

    for (int i = 1; i < argc; ++i) {
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

        // Triangulate all polygon faces to ensure correct area calculation
        CGAL::Polygon_mesh_processing::triangulate_faces(mesh);

        const bool is_closed = CGAL::is_closed(mesh);
        const double area = pcg::geom::mesh_surface_area(mesh);

        if (as_json) {
            std::cout << "{\n";
            std::cout << "  \"file\": \"" << path.string() << "\",\n";
            std::cout << "  \"closed\": " << (is_closed ? "true" : "false") << ",\n";
            std::cout << "  \"area\": " << area << "\n";
            std::cout << "}\n";
        } else {
            std::cout << path << "\n";
            std::cout << "  closed: " << (is_closed ? "true" : "false") << "\n";
            std::cout << "  area: " << area << "\n";
        }
    }
    return 0;
}
