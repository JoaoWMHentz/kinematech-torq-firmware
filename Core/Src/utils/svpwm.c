/*
 * svpwm.c
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Implementation of Space Vector PWM (SVPWM).
 *
 *  Math pipeline:
 *   1) dq -> αβ   : inverse Park transform (rotates voltages by θ).
 *   2) αβ -> abc  : project onto 3-phase system (Clarke inverse-equivalent).
 *   3) Centering  : add common-mode offset so that voltages fit into range.
 *   4) Clamp      : enforce limits [0, v_limit].
 *   5) Normalize  : divide by Vbus, get duty [0..1].
 *   6) Output     : write to TIM compare registers.
 */

#include <math.h>   // sinf, cosf, fminf, fmaxf
#include <utils/svpwm.h>

float clampf(float x, float a, float b) {
    return (x < a) ? a : ((x > b) ? b : x);
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

    Ua = clampf(Ua, 0.0f, v_limit);
    Ub = clampf(Ub, 0.0f, v_limit);
    Uc = clampf(Uc, 0.0f, v_limit);

    const float da = clampf(Ua / vbus, 0.0f, 1.0f);
    const float db = clampf(Ub / vbus, 0.0f, 1.0f);
    const float dc = clampf(Uc / vbus, 0.0f, 1.0f);

    /* ---------------------------------------------------------------
     * 6) Output to hardware timer
     * Scale duties to timer period (ARR) and write CCR registers.
     * This directly updates PWM outputs on TIM channels 1, 2, 3.
     * --------------------------------------------------------------- */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(da * period));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(db * period));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(dc * period));
}
