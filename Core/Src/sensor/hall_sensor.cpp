/*
 * hall_sensor.cpp
 *  Created on: Sep 21, 2025
 *      Author: Firmware Team
 */

#include "sensor/hall_sensor.hpp"

#include "definitions.h"

namespace kinematech {
namespace {
constexpr float kElectricalStep = TWO_PI / 6.0f; // 60 electrical degrees
constexpr float kMinDt = 1e-7f;
constexpr float kDefaultStale = 0.1f; // seconds before velocity decays to zero
} // namespace

HallSensor::HallSensor(const Pin& hall_a, const Pin& hall_b, const Pin& hall_c, uint8_t pole_pairs)
: pins_{ hall_a, hall_b, hall_c }, pole_pairs_(pole_pairs) {}

int HallSensor::init(float sample_hz) {
    if (pole_pairs_ == 0u) {
        return -1;
    }

    sample_period_ = (sample_hz > 0.f) ? (1.0f / sample_hz) : 0.f;
    mechanical_step_ = kElectricalStep / static_cast<float>(pole_pairs_);
    stale_timeout_s_ = (sample_period_ > 0.f) ? (5.0f * sample_period_) : kDefaultStale;

    last_sector_ = -1;
    mechanical_angle_wrapped_ = 0.f;
    mechanical_angle_unwrapped_ = 0.f;
    mech_velocity_ = 0.f;
    last_transition_time_s_ = nowSeconds();

    return update();
}

int HallSensor::update() {
    const uint8_t state = readState();
    last_state_ = state;

    if (state >= state_table_.size()) {
        return -2;
    }

    const int sector = state_table_[state];
    if (sector < 0) {
        // invalid hall combination
        return -3;
    }

    const float now = nowSeconds();

    if (last_sector_ < 0) {
        // First valid sample: seed angle state
        last_sector_ = sector;
        mechanical_angle_wrapped_ = static_cast<float>(sector) * mechanical_step_;
        mechanical_angle_unwrapped_ = mechanical_angle_wrapped_;
        last_transition_time_s_ = now;
        return 0;
    }

    if (sector != last_sector_) {
        int diff = sector - last_sector_;
        if (diff > 3) {
            diff -= 6;
        } else if (diff < -3) {
            diff += 6;
        }

        const float delta_mech = static_cast<float>(diff) * mechanical_step_;
        mechanical_angle_unwrapped_ += delta_mech;
        mechanical_angle_wrapped_ = wrapAngle(mechanical_angle_wrapped_ + delta_mech);

        float dt = now - last_transition_time_s_;
        if (dt <= kMinDt) {
            dt = (sample_period_ > kMinDt) ? sample_period_ : kMinDt;
        }
        mech_velocity_ = delta_mech / dt;

        last_transition_time_s_ = now;
        last_sector_ = sector;
    } else {
        if ((now - last_transition_time_s_) > stale_timeout_s_) {
            mech_velocity_ = 0.f;
        }
    }

    return 0;
}

int HallSensor::getAngle(float& theta_mech) {
    theta_mech = mechanical_angle_wrapped_;
    return 0;
}

int HallSensor::getVelocity(float& w_mech) {
    w_mech = mech_velocity_;
    return 0;
}

void HallSensor::setStateTable(const std::array<int8_t, 8>& table) {
    state_table_ = table;
}

uint8_t HallSensor::readState() const {
    uint8_t hall_a = HAL_GPIO_ReadPin(pins_[0].port, pins_[0].pin) == GPIO_PIN_SET ? 1u : 0u;
    uint8_t hall_b = HAL_GPIO_ReadPin(pins_[1].port, pins_[1].pin) == GPIO_PIN_SET ? 1u : 0u;
    uint8_t hall_c = HAL_GPIO_ReadPin(pins_[2].port, pins_[2].pin) == GPIO_PIN_SET ? 1u : 0u;

    return static_cast<uint8_t>(hall_a | (hall_b << 1) | (hall_c << 2));
}

float HallSensor::nowSeconds() {
    return static_cast<float>(HAL_GetTick()) / 1000.0f;
}

float HallSensor::wrapAngle(float angle) {
    if (angle >= TWO_PI) {
        angle -= TWO_PI;
    }
    if (angle < 0.f) {
        angle += TWO_PI;
    }
    return angle;
}

} // namespace kinematech
