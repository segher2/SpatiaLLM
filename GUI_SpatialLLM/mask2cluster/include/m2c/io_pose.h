#pragma once

#include <string>

#include "m2c/types.h"

namespace m2c {

// Load pose data from a JSON file, using only translation.x/y/z to populate Pose::C.
// Future implementation may rely on header-only nlohmann::json; rotation data is ignored.
Pose loadPoseJSON(const std::string& json_path);

}  // namespace m2c
