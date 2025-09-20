/*
 * foc_openloop.c
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#include <math.h>
#include "foc/foc_openloop.h"
#include "definitions.h"

static inline float clampf(float x, float a, float b){ return x<a?a:(x>b?b:x); }

void OpenLoopFOC_Init(OpenLoopFOC_t *s, float vbus, float loop_hz){
  s->vbus    = vbus;
  s->v_limit = SVPWM_LIMIT_K * vbus;
  s->uq_set  = OL_UQ_V;
  s->w_elec  = 2.0f * (float)M_PI * OL_FREQ_ELEC_HZ;
  s->theta   = 0.0f;
  s->dt      = 1.0f / loop_hz;
}

// Apply SVPWM to TIM1
static inline void svpwm(float Ud, float Uq, float theta,
                         float v_limit, float vbus,
                         uint32_t period, TIM_HandleTypeDef *htim)
{
  // Inverse Park transform
  float s = sinf(theta), c = cosf(theta);
  float Ualpha =  c*Ud - s*Uq;
  float Ubeta  =  s*Ud + c*Uq;

  // Clarke transform (αβ → abc)
  float Ua = Ualpha;
  float Ub = -0.5f*Ualpha + _SQRT3_2*Ubeta;
  float Uc = -0.5f*Ualpha - _SQRT3_2*Ubeta;

  // Center voltage shift
  float Umin = fminf(Ua, fminf(Ub, Uc));
  float Umax = fmaxf(Ua, fmaxf(Ub, Uc));
  float center = -0.5f*(Umax + Umin);

  Ua = clampf(Ua+center, 0.0f, v_limit);
  Ub = clampf(Ub+center, 0.0f, v_limit);
  Uc = clampf(Uc+center, 0.0f, v_limit);

  // Duty cycles [0..1]
  float da = clampf(Ua/vbus, 0.0f, 1.0f);
  float db = clampf(Ub/vbus, 0.0f, 1.0f);
  float dc = clampf(Uc/vbus, 0.0f, 1.0f);

  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(da*period));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(db*period));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(dc*period));
}

// One FOC open-loop step (called every PWM period)
void OpenLoopFOC_Step(OpenLoopFOC_t *s, TIM_HandleTypeDef *htim1){
  // Integrate electrical angle
  s->theta += s->w_elec * s->dt;
  if (s->theta >= TWO_PI) s->theta -= TWO_PI;

  // Open-loop: Ud = 0, Uq = constant
  const float Ud = 0.0f;
  const float Uq = clampf(s->uq_set, 0.0f, s->v_limit);

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim1);
  svpwm(Ud, Uq, s->theta, s->v_limit, s->vbus, arr, htim1);
}


