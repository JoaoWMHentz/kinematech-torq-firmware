/*
 * svpwm.h
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Space Vector PWM (SVPWM) API (C interface).
 *
 *  Math steps performed by svpwm():
 *    1) Inverse Park transform (dq -> αβ).
 *    2) Phase projection (αβ -> abc).
 *    3) Zero-sequence injection (center).
 *    4) Clamp to available voltage range [0, v_limit].
 *    5) Normalize by Vbus and update TIM CCR registers.
 */

#ifndef INC_MATH_SVPWM_H_
#define INC_MATH_SVPWM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"        // TIM_HandleTypeDef from CubeMX
#include "definitions.h" // constants like _SQRT3_2, SVPWM_LIMIT_K, etc.

/**
 * @brief Clamp float value into [a, b].
 */
float clampf(float x, float a, float b);

/**
 * @brief Compute and apply SVPWM duty cycles to timer channels.
 *
 * @param Ud       d-axis voltage [V] (0 in open-loop).
 * @param Uq       q-axis voltage [V].
 * @param theta    electrical angle [rad].
 * @param v_limit  max allowed voltage (after SVPWM scaling).
 * @param vbus     DC bus voltage [V].
 * @param period   timer ARR (PWM period in counts).
 * @param htim     pointer to HAL TIM handle.
 */
void svpwm(float Ud, float Uq, float theta, float v_limit, float vbus,
           uint32_t period, TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* INC_MATH_SVPWM_H_ */
