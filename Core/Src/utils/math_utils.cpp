/*
 * math_utils.cpp
 *
 *  Created on: Oct 1, 2025
 *      Author: joaoh
 */

#include "utils/math_utils.hpp"

#include "definitions.h"

extern "C" {
#include "arm_math.h"
}

#include <math.h>

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

void sincosFast(float angle, float& s, float& c) {
    arm_sin_cos_f32(angle, &s, &c);
}

void svpwm(float Ud, float Uq, float theta, float v_limit, float vbus,
           uint32_t period, TIM_HandleTypeDef *htim)
{

    const float s = sinf(theta);
    const float c = cosf(theta);
    const float Ualpha = c * Ud - s * Uq;
    const float Ubeta  = s * Ud + c * Uq;

    float Ua = Ualpha;
    float Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;   // √3/2 ≈ 0.8660254
    float Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

    const float Umin   = fminf(Ua, fminf(Ub, Uc));
    const float Umax   = fmaxf(Ua, fmaxf(Ub, Uc));
    const float center = -0.5f * (Umax + Umin);
    Ua += center;
    Ub += center;
    Uc += center;

    Ua = clamp(Ua, 0.0f, v_limit);
    Ub = clamp(Ub, 0.0f, v_limit);
    Uc = clamp(Uc, 0.0f, v_limit);

    const float da = clamp(Ua / vbus, 0.0f, 1.0f);
    const float db = clamp(Ub / vbus, 0.0f, 1.0f);
    const float dc = clamp(Uc / vbus, 0.0f, 1.0f);

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(da * period));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(db * period));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(dc * period));
}

} // namespace kinematech::utils
