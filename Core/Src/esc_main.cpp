/*
 * esc_main.cpp
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#include "main.h"
#include "definitions.h"

#include "motor/motor.hpp"
#include "sensor/sensor_null.hpp"
#include "driver/driver_openloop.hpp"
#include "driver/driver.hpp"

extern TIM_HandleTypeDef htim1;

// Global singletons (no heap)
static kinematech::Motor       g_motor({POLE_PAIRS, 0.f, 0.f, 0.f, 0.f});
static kinematech::NullSensor  g_sensor;
static kinematech::OpenLoopDriver g_driver(&htim1);

// Expose pointer for ISR
static kinematech::Driver* g_drv_ptr = &g_driver;

extern "C" void ESC_Main_Init(void){
  // Start PWM channels + complementary outputs
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  // Wire components
  g_drv_ptr->attachMotor(&g_motor);
  g_drv_ptr->attachSensor(&g_sensor);

  // Init
  g_drv_ptr->init(VBUS_V, PWM_FREQ_HZ);

  // Limits + controller (SimpleFOC-like)
  kinematech::LimitsCfg lim;
  lim.voltage_limit  = SVPWM_LIMIT_K * VBUS_V;
  lim.current_limit  = 0.f;
  lim.velocity_limit = 0.f;
  g_drv_ptr->setLimits(lim);

  kinematech::ControllerCfg cc;
  cc.motion_ctrl = kinematech::MotionControlType::Torque;
  cc.torque_ctrl = kinematech::TorqueControlType::Voltage;
  g_drv_ptr->setController(cc);

  // Initial target (Vq)
  g_drv_ptr->setTarget(OL_UQ_V);

  // Enable TIM1 update interrupt
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

extern "C" void ESC_Main_Loop(void){
  // You can change targets here (e.g., ramp Vq, CLI, UART, etc.)
  // g_drv_ptr->setTarget(new_vq);
}

// HAL ISR glue (C linkage)
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
  if (htim->Instance == TIM1 && g_drv_ptr){
    g_drv_ptr->step();
  }
}
