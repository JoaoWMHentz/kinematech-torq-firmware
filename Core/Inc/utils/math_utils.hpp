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

#include "tim.h"

namespace kinematech::utils {

float clamp(float value, float min_val, float max_val);

double clamp(double value, double min_val, double max_val);

float wrap_angle(float angle);

void sincosFast(float angle, float& s, float& c);

void svpwm(float Ud, float Uq, float theta, float v_limit, float vbus,
           uint32_t period, TIM_HandleTypeDef *htim);

} // namespace kinematech::utils

#endif /* INC_UTILS_MATH_UTILS_HPP_ */
