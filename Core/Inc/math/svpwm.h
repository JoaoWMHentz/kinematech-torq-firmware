/*
 * svpwm.h
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_MATH_SVPWM_H_
#define INC_MATH_SVPWM_H_

#include <cmath>
#include "main.h"
#include "definitions.h"

static inline float clampf(float x, float a, float b) {
	return x < a ? a : (x > b ? b : x);
}

static inline void svpwm(float Ud, float Uq, float theta, float v_limit, float vbus,
		uint32_t period, TIM_HandleTypeDef *htim) {
	const float s = sinf(theta);
	const float c = cosf(theta);
	const float Ualpha = c * Ud - s * Uq;
	const float Ubeta = s * Ud + c * Uq;

	float Ua = Ualpha;
	float Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
	float Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

	const float Umin = fminf(Ua, fminf(Ub, Uc));
	const float Umax = fmaxf(Ua, fmaxf(Ub, Uc));
	const float center = -0.5f * (Umax + Umin);

	Ua = clampf(Ua + center, 0.f, v_limit);
	Ub = clampf(Ub + center, 0.f, v_limit);
	Uc = clampf(Uc + center, 0.f, v_limit);

	const float da = clampf(Ua / vbus, 0.f, 1.f);
	const float db = clampf(Ub / vbus, 0.f, 1.f);
	const float dc = clampf(Uc / vbus, 0.f, 1.f);

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,
			static_cast<uint32_t>(da * period));
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2,
			static_cast<uint32_t>(db * period));
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3,
			static_cast<uint32_t>(dc * period));
}

#endif /* INC_MATH_SVPWM_H_ */
