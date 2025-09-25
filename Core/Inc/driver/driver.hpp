/*
 * driver.hpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Abstract base class that represents a motor driver strategy.
 *  Concrete implementations (e.g., open-loop SVPWM) inherit from this
 *  interface to gain common wiring, configuration, and scheduling hooks.
 */

#ifndef INC_DRIVER_DRIVER_H_
#define INC_DRIVER_DRIVER_H_

#pragma once

#include <cstdint>

#include "config/ctrl_enum.hpp"

namespace kinematech {

class Motor;   // Forward declaration (defined in motor/motor.hpp)
class Sensor;  // Forward declaration (defined in sensor/sensor.hpp)

/**
 * @class Driver
 * @brief Common interface for ESC driver implementations.
 *
 * The Driver abstraction owns references to the motor and sensor objects,
 * stores configuration/state shared by all strategies, and exposes the
 * periodic step() function that is executed at the PWM interrupt rate.
 */
class Driver {
public:
    virtual ~Driver() = default;

    /**
     * @brief Attach the motor model used by the driver.
     */
    virtual void attachMotor(Motor* m) {
        motor_ = m;
    }

    /**
     * @brief Attach the sensor providing rotor position feedback.
     */
    virtual void attachSensor(Sensor* s) {
        sensor_ = s;
    }

    /**
     * @brief Initialize driver resources.
     * @param vbus    DC bus voltage [V].
     * @param loop_hz Control loop frequency [Hz].
     * @return Implementation-specific status code (0 on success).
     */
    virtual int init(float vbus, float loop_hz) = 0;

    /**
     * @brief Update operating limits (voltage/current/velocity caps).
     */
    virtual void setLimits(const LimitsCfg& lim) {
        limits_ = lim;
    }

    /**
     * @brief Apply controller configuration (motion + torque control).
     */
    virtual void setController(const ControllerCfg& cc) {
        ctrl_ = cc;
    }

    /**
     * @brief Set the target command interpreted by the driver.
     */
    virtual void setTarget(float target) {
        target_ = target;
    }

    /**
     * @brief Feed latest phase current samples [A].
     */
    virtual void feedPhaseCurrents(float, float, float) {}

    /**
     * @brief Perform one control update.
     *
     * Called from the PWM interrupt (or main loop) at loop_hz rate.
     */
    virtual void step() = 0;

    // Accessors -----------------------------------------------------------------
    inline float vbus() const {
        return vbus_;
    }

    inline float dt() const {
        return dt_;
    }

    inline const LimitsCfg& limits() const {
        return limits_;
    }

    inline const ControllerCfg& ctrl() const {
        return ctrl_;
    }

protected:
    Motor* motor_ { nullptr };   ///< Attached motor description
    Sensor* sensor_ { nullptr }; ///< Attached sensor

    LimitsCfg limits_ { };       ///< Operating limits
    ControllerCfg ctrl_ { };     ///< Control strategy selection

    float target_ { 0.f };       ///< Current target command
    float vbus_ { 0.f };         ///< DC bus voltage [V]
    float dt_ { 0.f };           ///< Control loop period [s]
};

} // namespace kinematech

#endif /* INC_DRIVER_DRIVER_H_ */
