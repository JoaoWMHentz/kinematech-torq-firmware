/*
 * esc_main.c
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#include "esc_main.h"
#include "definitions.h"
#include "foc/foc_openloop.h"

// Open-loop FOC state
static OpenLoopFOC_t ol;

void ESC_Main_Init(void)
{
  // Start PWM channels + complementary outputs
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  // Initialize open-loop control
  OpenLoopFOC_Init(&ol, VBUS_V, PWM_FREQ_HZ);

  // Enable TIM1 update interrupt
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

void ESC_Main_Loop(void)
{
  // Nothing here yet: everything runs in TIM1 interrupt
  // You can add commands, ramps, serial interface, etc.
}

// TIM1 update ISR callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1){
    OpenLoopFOC_Step(&ol, &htim1);
  }
}
