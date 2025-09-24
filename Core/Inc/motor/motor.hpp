/*
 * motor.hpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Basic motor description used by the driver layer. The struct captures
 *  static parameters such as pole pairs and phase resistance, while the
 *  Motor class stores optional cached state that can be shared across
 *  control components.
 */

#ifndef INC_MOTOR_MOTOR_HPP_
#define INC_MOTOR_MOTOR_HPP_

#pragma once

#include <cstdint>

namespace kinematech {

/// Motor configuration values gathered from datasheets or measurement.
struct MotorCfg {
    uint8_t pole_pairs { 1 };
    float phase_resistance { 0.f };
    float phase_inductance { 0.f };
    float kv_rpm_per_v { 0.f };
    float kt_Nm_per_A { 0.f };
};

/**
 * @class Motor
 * @brief Lightweight holder for motor configuration and cached state.
 */
class Motor {
public:
    explicit Motor(const MotorCfg& cfg)
    : cfg_(cfg) {}

    /// @return Number of electrical pole pairs.
    inline uint8_t polePairs() const {
        return cfg_.pole_pairs;
    }

    /// @return Immutable view of the configuration data.
    inline const MotorCfg& cfg() const {
        return cfg_;
    }

    // Cached state (optional, filled by higher layers)
    float electrical_angle { 0.f };
    float mechanical_angle { 0.f };
    float mech_velocity { 0.f };

    /// Convert mechanical to electrical angle using pole pairs.
    static inline float elecFromMech(const Motor& m, float theta_mech) {
        return theta_mech * m.cfg_.pole_pairs;
    }

private:
    MotorCfg cfg_;
};

} // namespace kinematech

#endif /* INC_MOTOR_MOTOR_HPP_ */
