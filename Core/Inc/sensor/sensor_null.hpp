/*
 * sensor_null.hpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Trivial "null" sensor used for open-loop operation. It simply echoes
 *  values that are written into its public fields so that higher layers can
 *  simulate sensor feedback without hardware.
 */

#ifndef INC_SENSOR_SENSOR_NULL_HPP_
#define INC_SENSOR_SENSOR_NULL_HPP_

#pragma once

#include "sensor.hpp"

namespace kinematech {

/**
 * @class NullSensor
 * @brief Stub sensor that returns externally provided angle/velocity.
 */
class NullSensor final : public Sensor {
public:
    int init(float) override { return 0; }
    int update() override { return 0; }
    int getAngle(float& th) override {
        th = angle_mech;
        return 0;
    }
    int getVelocity(float& w) override {
        w = vel_mech;
        return 0;
    }

    float angle_mech { 0.f }; ///< Cached mechanical angle [rad]
    float vel_mech { 0.f };   ///< Cached mechanical velocity [rad/s]
};

} // namespace kinematech

#endif /* INC_SENSOR_SENSOR_NULL_HPP_ */
