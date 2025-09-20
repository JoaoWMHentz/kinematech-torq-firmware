/*
 * foc_openloop.h
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_FOC_FOC_OPENLOOP_H_
#define INC_FOC_FOC_OPENLOOP_H_

#include "stm32g4xx_hal.h"

typedef struct {
	float vbus, v_limit;
	float uq_set;     // q-axis voltage reference
	float w_elec;     // electrical angular velocity [rad/s]
	float theta;      // electrical angle [rad]
	float dt;         // loop period [s]
} OpenLoopFOC_t;

// Initialize open-loop FOC structure
void OpenLoopFOC_Init(OpenLoopFOC_t *s, float vbus, float loop_hz);

// Execute one open-loop step (update duty cycles)
void OpenLoopFOC_Step(OpenLoopFOC_t *s, TIM_HandleTypeDef *htim1);

#endif /* INC_FOC_FOC_OPENLOOP_H_ */
