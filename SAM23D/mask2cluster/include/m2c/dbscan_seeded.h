#pragma once

#include <vector>

namespace m2c {

struct Cluster {
	std::vector<int> indices;
	float diameter = 0.0f;
};

}  // namespace m2c
