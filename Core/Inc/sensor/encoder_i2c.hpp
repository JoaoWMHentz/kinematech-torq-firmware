/*
 * encoder_i2c.hpp
 *  Created on: Sep 21, 2025
 *      Author: Firmware Team
 *
 *  Skeleton implementation for I2C-based encoders. Provides a Sensor
 *  compliant interface so that the control stack can be wired before the
 *  concrete device driver is available.
 */

#ifndef INC_SENSOR_ENCODER_I2C_HPP_
#define INC_SENSOR_ENCODER_I2C_HPP_

#pragma once

#include <cstdint>

#include "sensor.hpp"

extern "C" {
#include "stm32g4xx_hal.h"
}

namespace kinematech {

struct I2cEncoderConfig {
    I2C_HandleTypeDef* bus { nullptr }; ///< HAL I2C handle
    uint8_t address { 0x00u };          ///< 7-bit device address
};

/**
 * @class I2cEncoder
 * @brief Placeholder for future I2C encoder implementations.
 */
class I2cEncoder : public Sensor {
public:
    explicit I2cEncoder(const I2cEncoderConfig& cfg);

    void setConfig(const I2cEncoderConfig& cfg);

    int init(float sample_hz) override;
    int update() override;
    int getAngle(float& theta_mech) override;
    int getVelocity(float& w_mech) override;

private:
    I2cEncoderConfig cfg_ { };
    float sample_period_ { 0.f };
    float theta_mech_ { 0.f };
    float velocity_mech_ { 0.f };
};

} // namespace kinematech

#endif /* INC_SENSOR_ENCODER_I2C_HPP_ */
