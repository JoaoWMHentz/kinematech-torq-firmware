/*
 * hall_sensor.hpp
 *  Created on: Sep 21, 2025
 *      Author: Firmware Team
 *
 *  Digital hall sensor implementation compatible with the SimpleFOC-style
 *  Sensor interface. Converts the three-phase hall pattern into mechanical
 *  angle/velocity estimates consumed by the control loop.
 */

#ifndef INC_SENSOR_HALL_SENSOR_HPP_
#define INC_SENSOR_HALL_SENSOR_HPP_

#pragma once

#include <array>
#include <cstdint>

#include "sensor.hpp"

extern "C" {
#include "stm32g4xx_hal.h"
}

namespace kinematech {

/**
 * @class HallSensor
 * @brief Rotor angle sensor backed by three digital hall inputs.
 */
class HallSensor final : public Sensor {
public:
    /**
     * @brief Helper describing a GPIO pin used by the hall sensor.
     */
    struct Pin {
        GPIO_TypeDef* port { nullptr }; ///< GPIO port instance
        uint16_t pin { 0u };            ///< GPIO pin mask
    };

    /**
     * @brief Construct sensor with explicit GPIO pin mapping.
     * @param hall_a Digital input for phase A.
     * @param hall_b Digital input for phase B.
     * @param hall_c Digital input for phase C.
     * @param pole_pairs Motor pole pairs (used for mech↔elec conversion).
     */
    HallSensor(const Pin& hall_a, const Pin& hall_b, const Pin& hall_c, uint8_t pole_pairs);

    int init(float sample_hz) override;
    int update() override;
    int getAngle(float& theta_mech) override;
    int getVelocity(float& w_mech) override;

    /**
     * @brief Override the hall state → electrical sector lookup table.
     *        Array indices correspond to the 3-bit hall state (A | B<<1 | C<<2).
     */
    void setStateTable(const std::array<int8_t, 8>& table);

    /**
     * @return Last raw hall state sampled.
     */
    inline uint8_t rawState() const { return last_state_; }

    /**
     * @return Most recent decoded electrical sector (0..5) or -1 if unknown.
     */
    inline int lastSector() const { return last_sector_; }

    /**
     * @return Absolute mechanical angle accumulated across rotations [rad].
     */
    inline float absoluteAngle() const { return mechanical_angle_unwrapped_; }

private:
    uint8_t readState() const;
    static float nowSeconds();
    static float wrapAngle(float angle);

    Pin pins_[3];
    uint8_t pole_pairs_ { 0u };
    float sample_period_ { 0.f };
    float mechanical_step_ { 0.f };
    std::array<int8_t, 8> state_table_ { { -1, 0, 4, 5, 2, 1, 3, -1 } };

    int last_sector_ { -1 };
    uint8_t last_state_ { 0u };
    float mechanical_angle_wrapped_ { 0.f };
    float mechanical_angle_unwrapped_ { 0.f };
    float mech_velocity_ { 0.f };
    float last_transition_time_s_ { 0.f };
    float stale_timeout_s_ { 0.0f };
};

} // namespace kinematech

#endif /* INC_SENSOR_HALL_SENSOR_HPP_ */
