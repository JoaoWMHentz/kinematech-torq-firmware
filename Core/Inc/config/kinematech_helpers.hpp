/*
 * kinematech_helpers.hpp
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_CONFIG_KINEMATECH_HELPERS_HPP_
#define INC_CONFIG_KINEMATECH_HELPERS_HPP_

#pragma once
#include "driver/driver.hpp"      // kinematech::Driver
#include "config/ctrl_enum.hpp"   // kinematech::{MotionControlType,TorqueControlType}

namespace kinematech {

// SimpleFOC-like helper wrappers
inline void KT_setVoltageLimit(Driver& d, float vlim){
  auto lim = d.limits(); lim.voltage_limit = vlim; d.setLimits(lim);
}
inline void KT_setVelocityLimit(Driver& d, float wlim){
  auto lim = d.limits(); lim.velocity_limit = wlim; d.setLimits(lim);
}
inline void KT_setTarget(Driver& d, float target){ d.setTarget(target); }
inline void KT_setController(Driver& d, MotionControlType m){
  auto cc = d.ctrl(); cc.motion_ctrl = m; d.setController(cc);
}
inline void KT_setTorqueController(Driver& d, TorqueControlType t){
  auto cc = d.ctrl(); cc.torque_ctrl = t; d.setController(cc);
}

} // namespace kinematech

#endif /* INC_CONFIG_KINEMATECH_HELPERS_HPP_ */
