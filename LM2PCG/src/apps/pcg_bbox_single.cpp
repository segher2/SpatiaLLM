// pcg_bbox_single.cpp
// Compute UOBB for a single point cloud file
// Usage: pcg_bbox_single <input.ply> <output_uobb.ply>
// Outputs: UOBB PLY file + JSON with geometry parameters to stdout

#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "pcg/bbox.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.ply> <output_uobb.ply>\n";
        std::cerr << "Computes upright oriented bounding box for input point cloud\n";
        std::cerr << "Outputs UOBB as PLY mesh and prints geometry parameters as JSON\n";
        return 1;
    }

    const std::string input_path = argv[1];
    const std::string output_path = argv[2];

    try {
        // Load input point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_path, *cloud) < 0) {
            std::cerr << "Error: Failed to load point cloud from " << input_path << "\n";
            return 1;
        }

        if (cloud->empty()) {
            std::cerr << "Error: Point cloud is empty\n";
            return 1;
        }

        // Create indices for all points
        std::vector<int> indices(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            indices[i] = static_cast<int>(i);
        }

        // Compute UOBB
        pcg::UOBB box = pcg::compute_uobb(cloud, indices);

        // Save UOBB as PLY mesh
        if (!pcg::save_uobb_ply(output_path, box)) {
            std::cerr << "Error: Failed to save UOBB to " << output_path << "\n";
            return 1;
        }

        // Output geometry parameters as JSON to stdout
        std::cout << "{\n";
        std::cout << "  \"center_x\": " << box.center.x() << ",\n";
        std::cout << "  \"center_y\": " << box.center.y() << ",\n";
        std::cout << "  \"center_z\": " << box.center.z() << ",\n";
        std::cout << "  \"size_x\": " << box.size.x() << ",\n";
        std::cout << "  \"size_y\": " << box.size.y() << ",\n";
        std::cout << "  \"size_z\": " << box.size.z() << ",\n";
        std::cout << "  \"yaw_rad\": " << box.yaw << "\n";
        std::cout << "}\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
