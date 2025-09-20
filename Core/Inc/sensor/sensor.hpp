/*
 * sensor.hpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Abstract interface for rotor position sensors. Concrete implementations
 *  (magnetic encoder, hall sensor, etc.) provide the runtime angle and
 *  velocity estimates consumed by the control loops.
 */

#ifndef INC_SENSOR_SENSOR_HPP_
#define INC_SENSOR_SENSOR_HPP_

#pragma once

namespace kinematech {

/**
 * @class Sensor
 * @brief Base class for all rotor sensor implementations.
 */
class Sensor {
public:
    virtual ~Sensor() = default;

    /// Initialize sensor hardware.
    virtual int init(float sample_hz) = 0;

    /// Refresh cached readings (if needed by the implementation).
    virtual int update() = 0;

    /// Obtain the mechanical angle [rad].
    virtual int getAngle(float& theta_mech) = 0;

    /// Obtain the mechanical velocity [rad/s].
    virtual int getVelocity(float& w_mech) = 0;
};

} // namespace kinematech

#endif /* INC_SENSOR_SENSOR_HPP_ */
