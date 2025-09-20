/*
 * sensor.h
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_SENSOR_SENSOR_HPP_
#define INC_SENSOR_SENSOR_HPP_

#pragma once

namespace kinematech {

// Abstract interface for sensors
class Sensor {
public:
  virtual ~Sensor() = default;
  virtual int  init(float sample_hz) = 0;
  virtual int  update() = 0;
  virtual int  getAngle(float& theta_mech) = 0;   // [rad]
  virtual int  getVelocity(float& w_mech) = 0;    // [rad/s]
};

} // namespace kinematech

#endif /* INC_SENSOR_SENSOR_H_ */
