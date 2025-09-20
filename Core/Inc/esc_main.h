/*
 * esc_main.h
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_ESC_MAIN_H_
#define INC_ESC_MAIN_H_

#include "stm32g4xx_hal.h"

// Initialize ESC application
void ESC_Main_Init(void);

// Application loop (executed in while(1))
void ESC_Main_Loop(void);

// TIM1 handle (created by Cube in main.c)
extern TIM_HandleTypeDef htim1;

#endif /* INC_ESC_MAIN_H_ */
