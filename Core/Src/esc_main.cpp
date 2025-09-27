/*
 * esc_main.cpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  High-level ESC application wiring. This module glues together the
 *  hardware timer, motor description, hall sensor, and the open-loop
 *  driver used for bring-up experiments. The flow mirrors the previous
 *  closed-loop setup but skips the cascaded PI layers:
 *   1) Instantiate global singletons (no heap usage).
 *   2) Start PWM outputs and complementary channels.
 *   3) Configure driver limits and seed the open-loop commands.
 *   4) Provide HAL callbacks that delegate to the C++ driver logic.
 */

#include "main.h"
#include "definitions.h"
#include "esc_api.h"

#include <cstdio>

#include "motor/motor.hpp"
#include "sensor/hall_sensor.hpp"
#include "sensor/encoder_i2c.hpp" // stub for future I2C encoder integration
#include "driver/driver_openloop.hpp"
#include "driver/driver.hpp"

namespace kinematech {

extern "C" {
extern TIM_HandleTypeDef htim1;
}

namespace {
constexpr uint32_t kHallPrintPeriodMs = 200U; // 10 Hz update rate
constexpr float kRadToMillirad = 1000.0f;
}

// -----------------------------------------------------------------------------
// Global singletons (avoid dynamic allocation in the interrupt context)
static Motor g_motor({ POLE_PAIRS, 0.f, 0.f, 0.f, 0.f });
static uint32_t last_print_ms = 0;
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
     * 4) Configure operating limits and seed open-loop commands
     * --------------------------------------------------------------- */
    LimitsCfg lim {};
    lim.voltage_limit = SVPWM_LIMIT_K * VBUS_V;
    lim.current_limit = 0.0f;
    lim.velocity_limit = 0.0f;
    g_drv_ptr->setLimits(lim);

    ControllerCfg cc {};
    cc.motion_ctrl = MotionControlType::Torque;
    cc.torque_ctrl = TorqueControlType::Voltage;
    g_drv_ptr->setController(cc);

    /* ---------------------------------------------------------------
     * 5) Prime driver state and enable PWM interrupt
     * - Seed voltage + electrical speed for open-loop spinning.
     * - Enable TIM1 update interrupt so the driver::step runs each cycle.
     * --------------------------------------------------------------- */
    g_driver.setUq(OL_UQ_V);
    g_driver.setElectricalSpeed(TWO_PI * OL_FREQ_ELEC_HZ);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

extern "C" void ESC_Main_Loop(void) {
    /* ---------------------------------------------------------------
     * Background hook.
     * Update targets, poll communication, etc. (currently empty).
     * --------------------------------------------------------------- */

// simple test command

    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_print_ms) >= kHallPrintPeriodMs && true) {
		//
        last_print_ms = now_ms;

        float theta_mech = 0.f;
        const int angle_status = g_sensor.getAngle(theta_mech);
        if (angle_status != 0) {
            theta_mech = 0.f;
        }

        float vel_mech = 0.f;
        const int velocity_status = g_sensor.getVelocity(vel_mech);

        const uint8_t raw = g_sensor.rawState();
        const int sector = g_sensor.lastSector();
        const float theta_abs = g_sensor.absoluteAngle();

        const long theta_mrad = static_cast<long>(theta_mech * kRadToMillirad);
        const long theta_abs_mrad = static_cast<long>(theta_abs * kRadToMillirad);
        const long vel_mrad = static_cast<long>(vel_mech * kRadToMillirad);

        std::printf(
            "HALL raw=0x%02X sector=%d theta=%ld mrad abs=%ld mrad vel=%ld mrad/s teste \r\n",
            static_cast<unsigned int>(raw),
            sector,
            theta_mrad,
            theta_abs_mrad,
			vel_mrad);
    }
}
extern "C" void ESC_SetVelocityTarget(float velocity_rad_s) {
    const float w_elec = velocity_rad_s * static_cast<float>(POLE_PAIRS);
    g_driver.setElectricalSpeed(w_elec);
}

extern "C" void ESC_SetTorqueTarget(float torque_cmd) {
    g_driver.setUq(torque_cmd);
}

extern "C" void ESC_SetAngleTarget(float angle_rad) {
    (void)angle_rad; // not supported in open-loop mode
}


// HAL ISR glue (C linkage)
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM1 && g_drv_ptr) {
        g_drv_ptr->step();
    }
}

} // namespace kinematech
