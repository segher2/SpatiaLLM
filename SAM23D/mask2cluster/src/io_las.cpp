#include "m2c/io_las.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#ifdef M2C_HAS_PDAL
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#endif

namespace m2c {
namespace {

std::string toLower(const std::string& s) {
  std::string result = s;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return result;
}

std::string extensionOf(const std::string& path) {
  const auto pos = path.find_last_of('.');
  if (pos == std::string::npos) {
    return std::string();
  }
  return toLower(path.substr(pos));
}

#ifdef M2C_HAS_PDAL
CloudT::Ptr loadLasViaPDAL(const std::string& path) {
  pdal::Options options;
  options.add("filename", path);

  pdal::LasReader reader;
  reader.setOptions(options);

  pdal::PointTable table;
  try {
    reader.prepare(table);
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("PDAL failed to prepare reader: ") + e.what());
  }

  pdal::PointViewSet viewSet;
  try {
    viewSet = reader.execute(table);
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("PDAL failed to execute reader: ") + e.what());
  }

  CloudT::Ptr cloud(new CloudT);
  for (const auto& viewPtr : viewSet) {
    if (!viewPtr) {
      continue;
    }
    const std::size_t size = viewPtr->size();
    cloud->reserve(cloud->size() + size);
    for (pdal::PointId idx = 0; idx < size; ++idx) {
      PointT point;
      point.x = static_cast<float>(viewPtr->getFieldAs<double>(pdal::Dimension::Id::X, idx));
      point.y = static_cast<float>(viewPtr->getFieldAs<double>(pdal::Dimension::Id::Y, idx));
      point.z = static_cast<float>(viewPtr->getFieldAs<double>(pdal::Dimension::Id::Z, idx));
      // Load RGB if available
      if (viewPtr->hasDim(pdal::Dimension::Id::Red)) {
        point.r = static_cast<std::uint8_t>(viewPtr->getFieldAs<std::uint16_t>(pdal::Dimension::Id::Red, idx) >> 8);
        point.g = static_cast<std::uint8_t>(viewPtr->getFieldAs<std::uint16_t>(pdal::Dimension::Id::Green, idx) >> 8);
        point.b = static_cast<std::uint8_t>(viewPtr->getFieldAs<std::uint16_t>(pdal::Dimension::Id::Blue, idx) >> 8);
      } else {
        point.r = point.g = point.b = 255;  // Default to white if no color
      }
      cloud->push_back(point);
    }
  }

  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}
#endif

}  // namespace

CloudT::Ptr loadAnyPointCloud(const std::string& path) {
  const std::string ext = extensionOf(path);
  if (ext == ".las") {
#ifdef M2C_HAS_PDAL
    return loadLasViaPDAL(path);
#else
    throw std::runtime_error(
        "LAS input requested but PDAL support was not built. Reconfigure with M2C_WITH_PDAL=ON and ensure PDAL is installed.");
#endif
  }

  if (ext == ".ply") {
    CloudT::Ptr cloud(new CloudT);
    const int ret = pcl::io::loadPLYFile(path, *cloud);
    if (ret < 0) {
      throw std::runtime_error("Failed to load PLY file: " + path);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
  }

  if (ext == ".pcd") {
    CloudT::Ptr cloud(new CloudT);
    const int ret = pcl::io::loadPCDFile(path, *cloud);
    if (ret < 0) {
      throw std::runtime_error("Failed to load PCD file: " + path);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
  }

  throw std::runtime_error("Unsupported point cloud extension: " + ext + " for path: " + path);
}

}  // namespace m2c
