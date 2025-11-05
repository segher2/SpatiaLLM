#include "m2c/io_pose.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace m2c {

Pose loadPoseJSON(const std::string& json_path) {
  std::ifstream input(json_path);
  if (!input) {
    std::ostringstream oss;
    oss << "Failed to open pose file: " << json_path;
    throw std::runtime_error(oss.str());
  }

  nlohmann::json doc;
  try {
    input >> doc;
  } catch (const std::exception& e) {
    std::ostringstream oss;
    oss << "Failed to parse JSON from " << json_path << ": " << e.what();
    throw std::runtime_error(oss.str());
  }

  if (!doc.contains("translation")) {
    std::ostringstream oss;
    oss << "Pose JSON missing 'translation' object: " << json_path;
    throw std::runtime_error(oss.str());
  }

  const nlohmann::json& translation = doc["translation"];
  Pose pose{};
  try {
    pose.C.x() = translation.at("x").get<float>();
    pose.C.y() = translation.at("y").get<float>();
    pose.C.z() = translation.at("z").get<float>();
  } catch (const std::exception& e) {
    std::ostringstream oss;
    oss << "Pose JSON missing required translation components in " << json_path << ": "
        << e.what();
    throw std::runtime_error(oss.str());
  }

  return pose;
}

}  // namespace m2c
