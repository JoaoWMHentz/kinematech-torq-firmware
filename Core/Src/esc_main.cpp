/*
 * esc_main.cpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  High-level ESC application wiring. This module glues together the
 *  hardware timer, motor description, sensor, and open-loop driver.
 *  The flow mirrors the structure used in svpwm.c/openloop_driver.cpp:
 *   1) Instantiate global singletons (no heap usage).
 *   2) Start PWM outputs and complementary channels.
 *   3) Configure driver limits + controller and set an initial target.
 *   4) Provide HAL callbacks that delegate to the C++ driver logic.
 */

#include "main.h"
#include "definitions.h"

#include "motor/motor.hpp"
#include "sensor/hall_sensor.hpp"
#include "sensor/encoder_i2c.hpp" // stub for future I2C encoder integration
#include "driver/driver_openloop.hpp"
#include "driver/driver.hpp"

namespace kinematech {

extern "C" {
extern TIM_HandleTypeDef htim1;
}

// -----------------------------------------------------------------------------
// Global singletons (avoid dynamic allocation in the interrupt context)
static Motor g_motor({ POLE_PAIRS, 0.f, 0.f, 0.f, 0.f });
static HallSensor g_sensor(
    { HALL_A_GPIO_Port, HALL_A_Pin },
    { HALL_B_GPIO_Port, HALL_B_Pin },
    { HALL_C_GPIO_Port, HALL_C_Pin },
    POLE_PAIRS);
static OpenLoopDriver g_driver(&htim1);

// Expose pointer for ISR dispatching.
static Driver* g_drv_ptr = &g_driver;

extern "C" void ESC_Main_Init(void) {
    /* ---------------------------------------------------------------
     * 1) Start PWM outputs (including low-side complements)
     * --------------------------------------------------------------- */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* ---------------------------------------------------------------
     * 2) Wire components together
     * Driver receives references to the statically allocated motor
     * and sensor objects.
     * --------------------------------------------------------------- */
    g_drv_ptr->attachMotor(&g_motor);
    g_drv_ptr->attachSensor(&g_sensor);

    /* ---------------------------------------------------------------
     * 3) Initialize driver with electrical parameters
     * vbus       : DC bus voltage.
     * PWM_FREQ_HZ: loop execution rate (equals PWM frequency here).
     * --------------------------------------------------------------- */
    g_drv_ptr->init(VBUS_V, PWM_FREQ_HZ);

    /* ---------------------------------------------------------------
     * 4) Configure operating limits and controller strategy
     * --------------------------------------------------------------- */
    LimitsCfg lim {};
    lim.voltage_limit = SVPWM_LIMIT_K * VBUS_V;
    lim.current_limit = 0.f;
    lim.velocity_limit = 0.f;
    g_drv_ptr->setLimits(lim);

    ControllerCfg cc {};
    cc.motion_ctrl = MotionControlType::Torque;
    cc.torque_ctrl = TorqueControlType::Voltage;
    g_drv_ptr->setController(cc);

    /* ---------------------------------------------------------------
     * 5) Prime driver state and enable PWM interrupt
     * - Set initial Vq target for open-loop drive.
     * - Enable TIM1 update interrupt so the driver::step runs each cycle.
     * --------------------------------------------------------------- */
    g_drv_ptr->setTarget(OL_UQ_V);

    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

extern "C" void ESC_Main_Loop(void) {
    /* ---------------------------------------------------------------
     * Background hook.
     * Update targets, poll communication, etc. (currently empty).
     * --------------------------------------------------------------- */
}

// HAL ISR glue (C linkage)
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM1 && g_drv_ptr) {
        g_drv_ptr->step();
    }
}

} // namespace kinematech
