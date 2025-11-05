#pragma once

#include "m2c/dbscan_seeded.h"

namespace m2c {

struct Validator {
	int minPtsTotal;     // Minimum cluster size required for acceptance.
	float maxDiameter;   // Maximum allowable diameter before rejection.

	bool pass(const Cluster& cluster) const;       // True when |S| >= minPtsTotal and diameter <= maxDiameter.
	bool sureNoise(const Cluster& cluster) const;  // True when the cluster is clearly underpopulated noise (|S| << minPtsTotal).
};

}  // namespace m2c
