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
    inline float absoluteAngle() const { return predicted_angle_unwrapped_; }

    // Accessor for ISR dispatch (singleton-style binding)
    static HallSensor* instance();
 
private:
    void onTimerEdgeIsr(uint32_t capture_ticks);

    uint8_t readState() const;
    static float wrapAngle(float angle);

    // Single-instance pointer for ISR dispatching
    static HallSensor* s_instance_;

    // Allow HAL capture callback to drive private ISR entrypoint
    friend void ::HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim);

    // Bound HAL timer handle (TIM8 Hall interface)
    TIM_HandleTypeDef* tim_ { nullptr };

    Pin pins_[3];
    uint8_t pole_pairs_ { 0u };
    float sample_period_ { 0.f };
    float mechanical_step_ { 0.f };
    std::array<int8_t, 8> state_table_ { { -1, 0, 4, 5, 2, 1, 3, -1 } };

    int last_sector_ { -1 };
    uint8_t last_state_ { 0u };
    float mechanical_angle_wrapped_ { 0.f };
    float mechanical_angle_unwrapped_ { 0.f };
    float predicted_angle_unwrapped_ { 0.f };
    float mech_velocity_ { 0.f };
    volatile uint32_t last_transition_tick_ms_ { 0u };
    float stale_timeout_s_ { 0.0f };
    float min_transition_dt_s_ { 0.f };
    uint32_t min_transition_ticks_ { 1u };

    // Timer-based timing info
    uint32_t last_capture_ticks_ { 0u };
    float tick_period_s_ { 0.f }; // seconds per timer tick
    float last_transition_dt_s_ { 0.f };
};

} // namespace kinematech

#endif /* INC_SENSOR_HALL_SENSOR_HPP_ */
