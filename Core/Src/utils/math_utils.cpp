/*
 * math_utils.cpp
 *
 *  Created on: Oct 1, 2025
 *      Author: joaoh
 */

#include "utils/math_utils.hpp"

#include "definitions.h"

namespace kinematech::utils {

float clamp(float value, float min_val, float max_val) {
	if (max_val < min_val) {
		const float tmp = min_val;
		min_val = max_val;
		max_val = tmp;
	}

	if (value < min_val) {
		return min_val;
	}
	if (value > max_val) {
		return max_val;
	}
	return value;
}

double clamp(double value, double min_val, double max_val) {
	if (max_val < min_val) {
		const double tmp = min_val;
		min_val = max_val;
		max_val = tmp;
	}

	if (value < min_val) {
		return min_val;
	}
	if (value > max_val) {
		return max_val;
	}
	return value;
}

float wrap_angle(float angle) {
	while (angle >= TWO_PI) {
		angle -= TWO_PI;
	}
	while (angle < 0.f) {
		angle += TWO_PI;
	}
	return angle;
}

} // namespace kinematech::utils
