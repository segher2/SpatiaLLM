// Command-line tool for mask2cluster
// Usage: mask2cluster_cli <input.ply> <pose.json> <output.ply> [options]

#include <iostream>
#include <string>
#include <cstdlib>

#include <pcl/io/ply_io.h>

#include "m2c/pipeline.h"
#include "m2c/io_las.h"
#include "m2c/io_pose.h"
#include "m2c/types.h"

void print_usage(const char* prog_name) {
    std::cerr << "Usage: " << prog_name << " <input.ply> <pose.json> <output.ply> [options]\n"
              << "\nArguments:\n"
              << "  input.ply    Input point cloud (PLY/PCD format)\n"
              << "  pose.json    Camera pose JSON file with translation field\n"
              << "  output.ply   Output refined cluster PLY file\n"
              << "\nOptions:\n"
              << "  --eps <float>        FEC clustering radius (default: 0.05)\n"
              << "  --n <float>          Size filter factor: floor(n * mean_size) (default: 0.3)\n"
              << "  --m <int>            Top-M nearest points for voting (default: 50)\n"
              << "  --min-pts <int>      Minimum points for core (default: 8)\n"
              << "  --min-total <int>    Minimum cluster size for validation (default: 100)\n"
              << "  --max-diameter <float>  Maximum cluster diameter (default: 0.0 = no limit)\n"
              << "\nExample:\n"
              << "  " << prog_name << " filtered.ply pose.json refined.ply --eps 0.05 --n 0.3 --m 50\n";
}

int main(int argc, char** argv) {
    if (argc < 4) {
        print_usage(argv[0]);
        return 1;
    }

    const std::string input_path = argv[1];
    const std::string pose_path = argv[2];
    const std::string output_path = argv[3];

    // Default parameters
    m2c::Params params;
    params.eps = 0.05f;
    params.n = 0.3f;
    params.m = 50;
    params.minPts_core = 8;
    params.minPts_total = 100;
    params.maxDiameter = 0.0f;  // no limit
    params.voxel = 0.0f;        // no downsampling
    params.maxPts = 0;          // no limit
    params.max_trials = 1;      // not used in FEC mode

    // Parse optional arguments
    for (int i = 4; i < argc; i += 2) {
        if (i + 1 >= argc) {
            std::cerr << "Error: Option " << argv[i] << " requires a value\n";
            return 1;
        }
        std::string opt = argv[i];
        std::string val = argv[i + 1];

        try {
            if (opt == "--eps") {
                params.eps = std::stof(val);
            } else if (opt == "--n") {
                params.n = std::stof(val);
            } else if (opt == "--m") {
                params.m = std::stoi(val);
            } else if (opt == "--min-pts") {
                params.minPts_core = std::stoi(val);
            } else if (opt == "--min-total") {
                params.minPts_total = std::stoi(val);
            } else if (opt == "--max-diameter") {
                params.maxDiameter = std::stof(val);
            } else {
                std::cerr << "Warning: Unknown option " << opt << "\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing " << opt << " value '" << val << "': " << e.what() << "\n";
            return 1;
        }
    }

    try {
        // Load input point cloud
        std::cout << "Loading point cloud from " << input_path << "...\n";
        m2c::CloudT::Ptr cloud = m2c::loadAnyPointCloud(input_path);
        if (!cloud || cloud->empty()) {
            std::cerr << "Error: Failed to load point cloud or cloud is empty\n";
            return 1;
        }
        std::cout << "  Loaded " << cloud->size() << " points\n";

        // Load camera pose
        std::cout << "Loading pose from " << pose_path << "...\n";
        m2c::Pose pose = m2c::loadPoseJSON(pose_path);
        std::cout << "  Camera position: [" << pose.C.x() << ", " << pose.C.y() << ", " << pose.C.z() << "]\n";

        // Run FEC-based cluster selection
        std::cout << "Running FEC cluster selection...\n";
        std::cout << "  Parameters: eps=" << params.eps 
                  << ", n=" << params.n 
                  << ", m=" << params.m 
                  << ", minPts_core=" << params.minPts_core << "\n";
        
        m2c::Result result = m2c::selectCluster(*cloud, pose, params);

        if (!result.found) {
            std::cerr << "Error: No suitable cluster found\n";
            return 1;
        }

        std::cout << "  Found cluster with " << result.cluster.indices.size() << " points\n";
        std::cout << "  Cluster diameter: " << result.cluster.diameter << " meters\n";

        // Extract selected cluster points
        m2c::CloudT::Ptr output_cloud(new m2c::CloudT);
        output_cloud->reserve(result.cluster.indices.size());
        for (int idx : result.cluster.indices) {
            if (idx >= 0 && static_cast<std::size_t>(idx) < cloud->size()) {
                output_cloud->push_back((*cloud)[idx]);
            }
        }

        output_cloud->width = static_cast<uint32_t>(output_cloud->size());
        output_cloud->height = 1;
        output_cloud->is_dense = false;

        // Save output as binary PLY
        std::cout << "Saving refined cluster to " << output_path << "...\n";
        if (pcl::io::savePLYFileBinary(output_path, *output_cloud) < 0) {
            std::cerr << "Error: Failed to save output PLY file\n";
            return 1;
        }

        std::cout << "Success! Saved " << output_cloud->size() << " points\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
