/*
 * driver_openloop.hpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Interface for the open-loop motor driver.
 *  This driver applies a commanded q-axis voltage (Vq) at a given electrical
 *  angular velocity, without feedback (no sensor, no closed-loop FOC).
 */

#ifndef INC_DRIVER_DRIVER_OPENLOOP_HPP_
#define INC_DRIVER_DRIVER_OPENLOOP_HPP_

#pragma once
#include "driver.hpp"
#include "definitions.h"
#include <cstdint>

// HAL includes (provides TIM_HandleTypeDef and timer macros)
extern "C" {
#include "stm32g4xx_hal.h"       // adjust to your MCU family: f4xx, h7xx, etc.
#include "stm32g4xx_hal_tim.h"
}

namespace kinematech {

/**
 * @class OpenLoopDriver
 * @brief Open-loop driver implementation using SVPWM.
 *
 * Responsibilities:
 * - Drive the motor with a fixed electrical frequency and Vq command.
 * - Convert angle + (Ud=0, Uq) into duty cycles using SVPWM.
 * - Provide simple setters for electrical speed and q-axis voltage.
 */
class OpenLoopDriver final : public Driver {
public:
    /**
     * @brief Construct driver bound to a specific HAL timer instance.
     * @param tim HAL timer handle used for PWM generation.
     */
    explicit OpenLoopDriver(TIM_HandleTypeDef* tim);

    /**
     * @brief Initialize driver with bus voltage and loop frequency.
     * @param vbus    DC bus voltage [V].
     * @param loop_hz Control loop frequency [Hz].
     * @return 0 if successful.
     */
    int init(float vbus, float loop_hz) override;

    /**
     * @brief Set operating limits (voltage, velocity caps).
     */
    void setLimits(const LimitsCfg& lim) override;

    /**
     * @brief Set controller configuration.
     * In open-loop, only Torque/Voltage control is meaningful.
     */
    void setController(const ControllerCfg& cc) override;

    /**
     * @brief Set target command.
     * In open-loop, interpreted as q-axis voltage (Vq).
     */
    void setTarget(float target) override;

    /**
     * @brief Perform one open-loop update step.
     * - Integrates electrical angle.
     * - Clamps Vq within limits.
     * - Calls SVPWM to update PWM duty cycles.
     */
    void step() override;

    /**
     * @brief Directly set electrical angular speed [rad/s].
     */
    void setElectricalSpeed(float w_elec_rad_s);

    /**
     * @brief Directly set q-axis voltage [V].
     * This also updates the target.
     */
    void setUq(float uq);

private:
    TIM_HandleTypeDef* htim_ { nullptr }; ///< HAL timer handle (PWM output)
    uint32_t period_ { 0 };               ///< Timer auto-reload value (PWM period)
    float theta_elec_ { 0.f };            ///< Electrical angle [rad]
    float w_elec_ { 0.f };                ///< Electrical angular velocity [rad/s]
    float uq_set_ { 0.f };                ///< Last set q-axis voltage [V]
    float v_limit_cache_ { 0.f };         ///< Cached voltage limit [V]
};

} // namespace kinematech

#endif /* INC_DRIVER_DRIVER_OPENLOOP_HPP_ */
