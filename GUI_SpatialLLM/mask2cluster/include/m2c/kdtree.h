#pragma once

#include <memory>
#include <vector>

#include "m2c/types.h"

namespace m2c {

class KD {
public:
	explicit KD(const CloudT& cloud);
	void radius(int idx, float r, std::vector<int>& out) const;

private:
	struct State;
	std::shared_ptr<State> state_;
};

}  // namespace m2c
