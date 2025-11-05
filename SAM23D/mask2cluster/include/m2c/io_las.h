#pragma once

#include <string>

#include "m2c/types.h"

namespace m2c {

// Load the masked point cloud from disk, prioritizing LAS via PDAL when available.
// Falls back to PLY/PCD ingestion through PCL IO when LAS support is disabled or missing.
// Implementations should return nullptr and surface descriptive errors when loading fails.
CloudT::Ptr loadAnyPointCloud(const std::string& path);

}  // namespace m2c
