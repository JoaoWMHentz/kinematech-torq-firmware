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
#include "tim.h"

#include <cstdio>

#include "motor/motor.hpp"
#include "sensor/hall_sensor.hpp"
#include "sensor/encoder_i2c.hpp" // stub for future I2C encoder integration
#include "driver/driver_openloop.hpp"
#include "driver/driver_closedloop.hpp"
#include "driver/driver.hpp"

namespace kinematech {

extern "C" {
extern TIM_HandleTypeDef htim1;
}

namespace {

void configureGateGpioLow(GPIO_TypeDef* port, uint16_t pin) {
    GPIO_InitTypeDef init { };
    init.Pin = pin;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_PULLDOWN;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &init);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void forceAllGatePinsLow() {
    configureGateGpioLow(PA_HIN_GPIO_Port, PA_HIN_Pin);
    configureGateGpioLow(PB_HIN_GPIO_Port, PB_HIN_Pin);
    configureGateGpioLow(PC_HIN_GPIO_Port, PC_HIN_Pin);
    configureGateGpioLow(PA_LIN_GPIO_Port, PA_LIN_Pin);
    configureGateGpioLow(PB_LIN_GPIO_Port, PB_LIN_Pin);
    configureGateGpioLow(PC_LIN_GPIO_Port, PC_LIN_Pin);
}

} // namespace
// -----------------------------------------------------------------------------
// Global singletons (avoid dynamic allocation in the interrupt context)
static Motor g_motor({ POLE_PAIRS, 0.f, 0.f, 0.f, 0.f });
static uint32_t last_print_ms = 0;
static HallSensor g_sensor(
    { HALL_A_GPIO_Port, HALL_A_Pin },
    { HALL_B_GPIO_Port, HALL_B_Pin },
    { HALL_C_GPIO_Port, HALL_C_Pin },
    POLE_PAIRS);
static ClosedLoopDriver g_cl_driver(&htim1);
// static OpenLoopDriver g_ol_driver(&htim1); // retain for quick open-loop fallback

// Expose pointer for ISR dispatching.
static Driver* g_drv_ptr = &g_cl_driver;

extern "C" void ESC_PrepareGateDrivers(void) {
    // Hold all gate-driver inputs low until the PWM module takes ownership.
    forceAllGatePinsLow();
}

extern "C" void ESC_Main_Init(void) {
    // Restore alternate function control before enabling PWM outputs.
    HAL_TIM_MspPostInit(&htim1);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);

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
    g_cl_driver.attachMotor(&g_motor);
    g_cl_driver.attachSensor(&g_sensor);
    g_drv_ptr = &g_cl_driver;
    // g_ol_driver.attachMotor(&g_motor);
    // g_ol_driver.attachSensor(&g_sensor);

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
    lim.voltage_limit = VBUS_V * SVPWM_LIMIT_K; // conservative voltage limit for closed-loop
    lim.current_limit = 0.0f;
    lim.velocity_limit = 0.0f;
    g_drv_ptr->setLimits(lim);

    ControllerCfg cc {};
    cc.motion_ctrl = MotionControlType::Velocity;
    cc.torque_ctrl = TorqueControlType::Voltage;
    g_drv_ptr->setController(cc);

    ClosedLoopDriver::PIConfig vel_cfg {};
    vel_cfg.kp = FOC_VEL_KP;
    vel_cfg.ki = FOC_VEL_KI;
    vel_cfg.limit = lim.voltage_limit;
    g_cl_driver.setVelocityGains(vel_cfg);

    ClosedLoopDriver::PIConfig pos_cfg {};
    pos_cfg.kp = FOC_POS_KP;
    pos_cfg.ki = FOC_POS_KI;
    pos_cfg.limit = lim.velocity_limit;
    g_cl_driver.setPositionGains(pos_cfg);

    ClosedLoopDriver::PIConfig curr_cfg {};
    curr_cfg.kp = FOC_CURR_KP;
    curr_cfg.ki = FOC_CURR_KI;
    curr_cfg.limit = lim.voltage_limit;
    g_cl_driver.setCurrentGains(curr_cfg);

    /* ---------------------------------------------------------------
     * 5) Prime driver state and enable PWM interrupt
     * - Closed-loop starts with zero targets; enable update interrupt.
     * - Keep open-loop seed handy for quick fallback testing.
     * --------------------------------------------------------------- */
    const float initial_velocity_mech = OL_FREQ_ELEC_HZ;
    g_cl_driver.setTarget(initial_velocity_mech);
    // g_ol_driver.init(VBUS_V, PWM_FREQ_HZ);
    // g_ol_driver.setLimits(lim);
    // g_ol_driver.setController(cc);
    // g_ol_driver.setUq(OL_UQ_V);
    // g_ol_driver.setElectricalSpeed(TWO_PI * OL_FREQ_ELEC_HZ);
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

    /* Service lower-rate sensor maintenance outside of the FOC ISR. */
    static uint32_t last_sensor_update_ms = 0U;
    if ((now_ms - last_sensor_update_ms) >= 1U) {
        g_sensor.update();
        last_sensor_update_ms = now_ms;
    }

    if ((now_ms - last_print_ms) >= HALL_PRINT_PERIOD && true) {
		//
        last_print_ms = now_ms;

        float theta_mech = 0.f;
        const int angle_status = g_sensor.getAngle(theta_mech);
        if (angle_status != 0) {
            theta_mech = 0.f;
        }

        float vel_mech = 0.f;
        g_sensor.getVelocity(vel_mech);

        const uint8_t raw = g_sensor.rawState();
        const int sector = g_sensor.lastSector();
        const float theta_abs = g_sensor.absoluteAngle();

        const long theta_mrad = static_cast<long>(theta_mech * KRAD_TO_MIRAD);
        const long theta_abs_mrad = static_cast<long>(theta_abs * KRAD_TO_MIRAD);
        const long vel_mrad = static_cast<long>(vel_mech * KRAD_TO_MIRAD);
        const long vel_rpm_x10 = static_cast<long>(vel_mech * (10.0f * KRAD_TO_RPM));
        const bool vel_rpm_negative = vel_rpm_x10 < 0;
        const long vel_rpm_abs_x10 = vel_rpm_negative ? -vel_rpm_x10 : vel_rpm_x10;
        const long vel_rpm_int = vel_rpm_abs_x10 / 10;
        const long vel_rpm_frac = vel_rpm_abs_x10 % 10;
        const char vel_rpm_sign = vel_rpm_negative ? '-' : '+';

        std::printf(
            "HALL raw=0x%02X sector=%d theta=%ld mrad abs=%ld mrad vel=%ld mrad/s (%c%ld.%01ld rpm) teste \r\n",
            static_cast<unsigned int>(raw),
            sector,
            theta_mrad,
            theta_abs_mrad,
			vel_mrad,
			vel_rpm_sign,
			vel_rpm_int,
			vel_rpm_frac);
    }
}
extern "C" void ESC_SetVelocityTarget(float velocity_rad_s) {
    if (!g_drv_ptr) {
        return;
    }
    if (g_drv_ptr == &g_cl_driver) {
        ControllerCfg cc = g_cl_driver.ctrl();
        cc.motion_ctrl = MotionControlType::Velocity;
        g_cl_driver.setController(cc);
        g_cl_driver.setTarget(velocity_rad_s);
    }
    // else if (g_drv_ptr == &g_ol_driver) {
    //     const float w_elec = velocity_rad_s * static_cast<float>(POLE_PAIRS);
    //     g_ol_driver.setElectricalSpeed(w_elec);
    // }
}

extern "C" void ESC_SetTorqueTarget(float torque_cmd) {
    if (!g_drv_ptr) {
        return;
    }
    if (g_drv_ptr == &g_cl_driver) {
        ControllerCfg cc = g_cl_driver.ctrl();
        cc.motion_ctrl = MotionControlType::Torque;
        cc.torque_ctrl = TorqueControlType::Voltage;
        g_cl_driver.setController(cc);
        g_cl_driver.setTarget(torque_cmd);
    }
    // else if (g_drv_ptr == &g_ol_driver) {
    //     g_ol_driver.setUq(torque_cmd);
    // }
}

extern "C" void ESC_SetAngleTarget(float angle_rad) {
    if (!g_drv_ptr) {
        return;
    }
    if (g_drv_ptr == &g_cl_driver) {
        ControllerCfg cc = g_cl_driver.ctrl();
        cc.motion_ctrl = MotionControlType::Angle;
        g_cl_driver.setController(cc);
        g_cl_driver.setTarget(angle_rad);
    }
    // else {
    //     (void)angle_rad; // not supported in open-loop mode
    // }
}


// HAL ISR glue (C linkage)
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM1 && g_drv_ptr) {
        g_drv_ptr->step();
    }
}

} // namespace kinematech
