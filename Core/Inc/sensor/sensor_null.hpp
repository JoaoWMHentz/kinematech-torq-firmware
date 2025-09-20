/*
 * sensor_null.hpp
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_SENSOR_SENSOR_NULL_HPP_
#define INC_SENSOR_SENSOR_NULL_HPP_

#pragma once
#include "sensor.hpp"

namespace kinematech {

// "Null" sensor: returns cached values you set externally (for open-loop)
class NullSensor final : public Sensor {
public:
  int init(float) override { return 0; }
  int update() override { return 0; }
  int getAngle(float& th) override { th = angle_mech; return 0; }
  int getVelocity(float& w) override { w = vel_mech;  return 0; }

  float angle_mech{0.f};
  float vel_mech{0.f};
};

} // namespace kinematech


#endif /* INC_SENSOR_SENSOR_NULL_HPP_ */
