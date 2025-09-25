
/*
 * driver_closedloop.hpp
 *  Created on: Nov 7, 2025
 *      Author: Firmware Team
 *
 *  Field-oriented closed-loop driver inspired by SimpleFOC.
 */

#ifndef INC_DRIVER_DRIVER_CLOSEDLOOP_HPP_
#define INC_DRIVER_DRIVER_CLOSEDLOOP_HPP_

#pragma once

#include <cstdint>

#include "driver.hpp"

extern "C" {
#include "stm32g4xx_hal.h"
}

namespace kinematech {

/**
 * @class ClosedLoopDriver
 * @brief Field-oriented control (FOC) strategy that mirrors SimpleFOC.
 *
 * The driver runs cascaded PI loops (position → velocity → torque/current)
 * on top of hall/encoder feedback, ultimately producing SVPWM voltages that
 * are written to the TIM1 bridge timer.
 */
class ClosedLoopDriver final : public Driver {
public:
    /**
     * @brief SimpleFOC-style PI configuration (gain + clamp).
     */
    struct PIConfig {
        float kp { 0.f };    ///< Proportional gain
        float ki { 0.f };    ///< Integral gain
        float limit { 0.f }; ///< Absolute output limit
    };

    /**
     * @brief Phase current samples expressed in the abc frame.
     */
    struct PhaseCurrents {
        float ia { 0.f };    ///< Phase A current [A]
        float ib { 0.f };    ///< Phase B current [A]
        float ic { 0.f };    ///< Phase C current [A]
    };

    /**
     * @brief Construct the driver bound to a PWM timer handle.
     */
    explicit ClosedLoopDriver(TIM_HandleTypeDef* tim);

    /**
     * @brief Prepare timers/sensors and seed the cascaded loops.
     *
     * @param vbus    DC bus voltage [V].
     * @param loop_hz Control loop frequency (PWM rate) [Hz].
     */
    int init(float vbus, float loop_hz) override;

    /**
     * @brief Propagate updated operating limits into the loops.
     */
    void setLimits(const LimitsCfg& lim) override;

    /**
     * @brief Store controller selection (torque/velocity/position).
     */
    void setController(const ControllerCfg& cc) override;

    /**
     * @brief Update the user target (torque, speed or angle depending on mode).
     */
    void setTarget(float target) override;

    /**
     * @brief Execute one closed-loop FOC iteration (sensor → PI → SVPWM).
     */
    void step() override;

    /**
     * @brief Runtime tuning helpers mirroring SimpleFOC setters.
     */
    void setVelocityGains(const PIConfig& cfg);
    void setPositionGains(const PIConfig& cfg);
    void setCurrentGains(const PIConfig& cfg);
    void setVelocityFilterCutoff(float cutoff_hz);
    void setSensorDirection(int8_t dir);
    void setElectricalZero(float offset_rad);

    /**
     * @brief Feed latest phase current samples (abc).
     */
    void feedPhaseCurrents(const PhaseCurrents& iabc);

    void feedPhaseCurrents(float ia, float ib, float ic) override;

    /**
     * @brief Accessors for diagnostics.
     */
    float electricalAngle() const;
    float shaftAngle() const;
    float shaftVelocity() const;
    float qVoltage() const;

private:
    struct PIController {
        float kp { 0.f };
        float ki { 0.f };
        float limit { 0.f };
        float integral { 0.f };

        float apply(float error, float dt);
        void reset();
    };

    struct DQValues {
        float d { 0.f };
        float q { 0.f };
    };

    // --- Internal helpers ---------------------------------------------------
    void sampleSensor();
    float unwrapAngle(float raw_angle);
    float wrapAngle(float angle) const;
    float clamp(float value, float min_val, float max_val) const;

    float computeTorqueSetpoint();
    float velocityLoop(float velocity_target);
    float positionLoop(float position_target);
    float torqueLoop(float torque_target);
    void applyFOC(float uq_cmd);
    void updateCurrentDQ(const PhaseCurrents& iabc);

    // --- Cached hardware references ----------------------------------------
    TIM_HandleTypeDef* htim_ { nullptr };
    uint32_t period_ { 0u };

    // --- Sensor state -------------------------------------------------------
    float theta_mech_raw_ { 0.f };
    float theta_mech_prev_raw_ { 0.f };
    float theta_mech_unwrapped_ { 0.f };
    float theta_mech_prev_unwrapped_ { 0.f };
    float theta_mech_delta_ { 0.f };
    float theta_elec_ { 0.f };
    float zero_elec_offset_ { 0.f };
    int8_t sensor_dir_ { 1 };

    // --- Velocity estimation ------------------------------------------------
    float velocity_meas_ { 0.f };
    float velocity_filtered_ { 0.f };
    float velocity_filter_alpha_ { 0.2f };

    // --- Control loop state -------------------------------------------------
    float torque_target_ { 0.f };
    float iq_target_ { 0.f };

    PIController velocity_pi_ {};
    PIController position_pi_ {};
    PIController current_pi_ {};

    bool current_valid_ { false };
    DQValues current_meas_ {};
    DQValues voltage_cmd_ {};

    float v_limit_cache_ { 0.f };
    float last_target_ { 0.f };
    bool sensor_ready_ { false };
};

} // namespace kinematech

#endif /* INC_DRIVER_DRIVER_CLOSEDLOOP_HPP_ */
