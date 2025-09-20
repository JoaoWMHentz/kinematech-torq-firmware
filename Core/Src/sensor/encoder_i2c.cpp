/*
 * encoder_i2c.cpp
 *  Created on: Sep 21, 2025
 *      Author: Firmware Team
 */

#include "sensor/encoder_i2c.hpp"

namespace kinematech {

I2cEncoder::I2cEncoder(const I2cEncoderConfig& cfg)
: cfg_(cfg) {}

void I2cEncoder::setConfig(const I2cEncoderConfig& cfg) {
    cfg_ = cfg;
}

int I2cEncoder::init(float sample_hz) {
    sample_period_ = (sample_hz > 0.f) ? (1.0f / sample_hz) : 0.f;
    theta_mech_ = 0.f;
    velocity_mech_ = 0.f;
    // Returning non-zero allows caller to detect missing wiring and retry later.
    return (cfg_.bus != nullptr) ? 0 : -1;
}

int I2cEncoder::update() {
    // Placeholder: concrete encoder drivers will populate angle/velocity here.
    return (cfg_.bus != nullptr) ? 0 : -1;
}

int I2cEncoder::getAngle(float& theta_mech) {
    theta_mech = theta_mech_;
    return (cfg_.bus != nullptr) ? 0 : -1;
}

int I2cEncoder::getVelocity(float& w_mech) {
    w_mech = velocity_mech_;
    return (cfg_.bus != nullptr) ? 0 : -1;
}

} // namespace kinematech
