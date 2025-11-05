#pragma once

#include "m2c/dbscan_seeded.h"
#include "m2c/types.h"
#include "m2c/validator.h"

namespace m2c {

struct Result {
	bool found = false;  // True when a qualifying cluster is produced.
	int trials = 0;      // Number of seed attempts made.
	Cluster cluster;     // Captured cluster (valid when found == true).
};

// Orchestrate FEC-based cluster selection around reference point C.
// Steps: run FEC with radius `eps`, discard clusters smaller than floor(n * mean_size),
// then select the cluster that has majority among the `m` nearest-to-C points (ties broken by total distance).
Result selectCluster(const CloudT& cloud, const Pose& pose, const Params& params);

}  // namespace m2c
