/*
 * math_utils.hpp
 *
 *  Created on: Oct 1, 2025
 *      Author: joaoh
 */

#ifndef INC_UTILS_MATH_UTILS_HPP_
#define INC_UTILS_MATH_UTILS_HPP_

#pragma once

#include <cstdint>

namespace kinematech::utils {

float clamp(float value, float min_val, float max_val);
double clamp(double value, double min_val, double max_val);
float wrap_angle(float angle);

} // namespace kinematech::utils

#endif /* INC_UTILS_MATH_UTILS_HPP_ */
