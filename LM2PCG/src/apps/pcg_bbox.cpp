#include "pcg/ply_io.hpp"
#include "pcg/params.hpp"

#include <Eigen/Core>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <cmath>
#include "pcg/bbox.hpp"

namespace fs = std::filesystem;

static Eigen::Vector3f compute_center_from_vertices(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& verts) {
    if (!verts || verts->empty()) return Eigen::Vector3f::Zero();
    Eigen::Vector3f acc(0.f, 0.f, 0.f);
    for (const auto& p : verts->points) acc += Eigen::Vector3f(p.x, p.y, p.z);
    acc /= static_cast<float>(verts->points.size());
    return acc;
}

static void print_usage() {
    std::cerr << "Usage:\n";
    std::cerr << "  pcg_bbox <bbox1_uobb.ply> <bbox2_uobb.ply>     # compute centers, vector and distance\n";
    std::cerr << "  pcg_bbox gen <out.ply> cx cy cz lx ly lz yaw_deg  # generate a test UOBB PLY\n";
    std::cerr << "  pcg_bbox point x y z <bbox_uobb.ply>             # point-to-bbox-center vector & distance\n";
}

static bool generate_uobb(const std::string& out_path,
                          float cx, float cy, float cz,
                          float lx, float ly, float lz,
                          float yaw_deg) {
    const float yaw = yaw_deg * static_cast<float>(M_PI) / 180.0f;
    pcg::UOBB box{};
    box.center = Eigen::Vector3f(cx, cy, cz);
    box.size   = Eigen::Vector3f(lx, ly, lz);
    box.yaw    = yaw;
    box.R.setIdentity();
    box.R(0,0) =  std::cos(yaw); box.R(1,0) = std::sin(yaw);
    box.R(0,1) = -std::sin(yaw); box.R(1,1) = std::cos(yaw);

    const Eigen::Vector3f e1 = box.R.col(0);
    const Eigen::Vector3f e2 = box.R.col(1);
    const Eigen::Vector3f ez(0.f,0.f,1.f);
    const Eigen::Vector3f h = 0.5f * box.size;
    std::array<Eigen::Vector3f,4> base{
        box.center + e1*h.x() + e2*h.y() - ez*h.z(),
        box.center - e1*h.x() + e2*h.y() - ez*h.z(),
        box.center - e1*h.x() - e2*h.y() - ez*h.z(),
        box.center + e1*h.x() - e2*h.y() - ez*h.z()
    };
    std::array<Eigen::Vector3f,4> top{
        base[0] + ez*box.size.z(),
        base[1] + ez*box.size.z(),
        base[2] + ez*box.size.z(),
        base[3] + ez*box.size.z()
    };
    box.corners = { base[0], base[1], base[2], base[3], top[0], top[1], top[2], top[3] };
    return pcg::save_uobb_ply(out_path, box);
}

int main(int argc, char** argv) {
    auto find_config_path = [](){
        const fs::path candidates[] = {
            fs::path("data/configs/default.yaml"),
            fs::path("../data/configs/default.yaml"),
            fs::path("../../data/configs/default.yaml")
        };
        for (const auto& p : candidates) if (fs::exists(p)) return p; return fs::path();
    };
    pcg::ParamsConfig cfg; std::string err;
    pcg::load_params_from_file(find_config_path().string(), cfg, &err);
    const bool as_json = cfg.json_output;
    if (argc >= 2 && std::string(argv[1]) == "gen") {
        // gen mode: pcg_bbox gen out.ply cx cy cz lx ly lz yaw_deg
        if (argc != 10) { // prog, gen, out, cx,cy,cz,lx,ly,lz,yaw => 10
            print_usage(); return 1;
        }
        const std::string out = argv[2];
        const float cx = std::stof(argv[3]);
        const float cy = std::stof(argv[4]);
        const float cz = std::stof(argv[5]);
        const float lx = std::stof(argv[6]);
        const float ly = std::stof(argv[7]);
        const float lz = std::stof(argv[8]);
        const float yaw_deg = std::stof(argv[9]);
        if (!generate_uobb(out, cx, cy, cz, lx, ly, lz, yaw_deg)) {
            if (as_json) {
                std::cout << "{\n  \"mode\": \"gen\", \"status\": \"failed\", \"file\": \"" << out << "\"\n}\n";
            } else {
                std::cerr << "Write failed: " << out << "\n";
            }
            return 2;
        }
        if (as_json) {
            std::cout << "{\n  \"mode\": \"gen\", \"status\": \"ok\", \"file\": \"" << out << "\"\n}\n";
        } else {
            std::cout << "Generated: " << out << "\n";
        }
        return 0;
    }

    if (argc >= 2 && std::string(argv[1]) == "point") {
        // point mode: pcg_bbox point x y z bbox_uobb.ply
        if (argc != 6) { print_usage(); return 1; }
        const float px = std::stof(argv[2]);
        const float py = std::stof(argv[3]);
        const float pz = std::stof(argv[4]);
        const fs::path box_path = argv[5];
        if (!fs::exists(box_path)) { if (as_json) {
                std::cout << "{\n  \"mode\": \"point\", \"status\": \"not_found\", \"file\": \"" << box_path.string() << "\"\n}\n";
            } else {
                std::cerr << "Error: file not found: " << box_path << "\n"; }
            return 2; }

        pcl::PointCloud<pcl::PointXYZ>::Ptr verts;
        if (!pcg::load_ply_xyz(box_path.string(), verts, &err)) {
            if (as_json) {
                std::cout << "{\n  \"mode\": \"point\", \"status\": \"read_failed\", \"error\": \"" << err << "\"\n}\n";
            } else {
                std::cerr << "Error reading PLY: " << err << "\n"; }
            return 3;
        }
        if (!verts || verts->empty()) { std::cerr << "Error: PLY has no vertices\n"; return 3; }

        const Eigen::Vector3f p(px, py, pz);
        const Eigen::Vector3f c = compute_center_from_vertices(verts);
        const Eigen::Vector3f v = c - p; // vector from point to bbox center
        const float d = v.norm();

        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout << std::setprecision(6);
        if (as_json) {
            std::cout << "{\n  \"mode\": \"point\",\n  \"point\": {\"x\": " << p.x() << ", \"y\": " << p.y() << ", \"z\": " << p.z() << "},\n  \"bbox_center\": {\"x\": " << c.x() << ", \"y\": " << c.y() << ", \"z\": " << c.z() << "},\n  \"vector_point_to_center\": {\"x\": " << v.x() << ", \"y\": " << v.y() << ", \"z\": " << v.z() << "},\n  \"distance\": " << d << "\n}\n";
        } else {
            std::cout << "point: " << p.x() << ", " << p.y() << ", " << p.z() << "\n";
            std::cout << "bbox_center: " << c.x() << ", " << c.y() << ", " << c.z() << "\n";
            std::cout << "vector_point_to_center: " << v.x() << ", " << v.y() << ", " << v.z() << "\n";
            std::cout << "distance: " << d << "\n";
        }
        return 0;
    }

    if (argc != 3) { print_usage(); return 1; }
    const fs::path box1_path = argv[1];
    const fs::path box2_path = argv[2];
    if (!fs::exists(box1_path)) { if (as_json) {
            std::cout << "{\n  \"mode\": \"pair\", \"status\": \"not_found\", \"file\": \"" << box1_path.string() << "\"\n}\n";
        } else {
            std::cerr << "Error: file not found: " << box1_path << "\n"; }
        return 2; }
    if (!fs::exists(box2_path)) { if (as_json) {
            std::cout << "{\n  \"mode\": \"pair\", \"status\": \"not_found\", \"file\": \"" << box2_path.string() << "\"\n}\n";
        } else {
            std::cerr << "Error: file not found: " << box2_path << "\n"; }
        return 2; }

    // Load vertices from the UOBB PLYs. Our minimal loader reads vertex positions (x,y,z).
    pcl::PointCloud<pcl::PointXYZ>::Ptr verts1, verts2;
    if (!pcg::load_ply_xyz(box1_path.string(), verts1, &err)) {
        if (as_json) { std::cout << "{\n  \"mode\": \"pair\", \"status\": \"read_failed\", \"which\": 1, \"error\": \"" << err << "\"\n}\n"; }
        else { std::cerr << "Error reading PLY #1: " << err << "\n"; }
        return 3;
    }
    if (!pcg::load_ply_xyz(box2_path.string(), verts2, &err)) {
        if (as_json) { std::cout << "{\n  \"mode\": \"pair\", \"status\": \"read_failed\", \"which\": 2, \"error\": \"" << err << "\"\n}\n"; }
        else { std::cerr << "Error reading PLY #2: " << err << "\n"; }
        return 3;
    }
    if (!verts1 || verts1->empty()) { if (as_json) { std::cout << "{\n  \"mode\": \"pair\", \"status\": \"empty\", \"which\": 1\n}\n"; } else { std::cerr << "Error: PLY #1 has no vertices\n"; } return 3; }
    if (!verts2 || verts2->empty()) { if (as_json) { std::cout << "{\n  \"mode\": \"pair\", \"status\": \"empty\", \"which\": 2\n}\n"; } else { std::cerr << "Error: PLY #2 has no vertices\n"; } return 3; }

    const Eigen::Vector3f c1 = compute_center_from_vertices(verts1);
    const Eigen::Vector3f c2 = compute_center_from_vertices(verts2);
    const Eigen::Vector3f v12 = c2 - c1;
    const float dist = v12.norm();

    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout << std::setprecision(6);

    if (as_json) {
        std::cout << "{\n  \"mode\": \"pair\",\n  \"center1\": {\"x\": " << c1.x() << ", \"y\": " << c1.y() << ", \"z\": " << c1.z() << "},\n  \"center2\": {\"x\": " << c2.x() << ", \"y\": " << c2.y() << ", \"z\": " << c2.z() << "},\n  \"vector_1_to_2\": {\"x\": " << v12.x() << ", \"y\": " << v12.y() << ", \"z\": " << v12.z() << "},\n  \"distance\": " << dist << "\n}\n";
    } else {
        std::cout << "center1: " << c1.x() << ", " << c1.y() << ", " << c1.z() << "\n";
        std::cout << "center2: " << c2.x() << ", " << c2.y() << ", " << c2.z() << "\n";
        std::cout << "vector_1_to_2: " << v12.x() << ", " << v12.y() << ", " << v12.z() << "\n";
        std::cout << "distance: " << dist << "\n";
    }

    return 0;
}
