/*
 * kinematech_helpers.hpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Tiny helper functions inspired by SimpleFOC's API. They provide
 *  user-friendly wrappers to configure the driver without exposing
 *  internal structs to higher level application code.
 */

#ifndef INC_CONFIG_KINEMATECH_HELPERS_HPP_
#define INC_CONFIG_KINEMATECH_HELPERS_HPP_

#pragma once
#ifndef KINEMATECH_CONFIG_HELPERS_HPP
#define KINEMATECH_CONFIG_HELPERS_HPP

#include "driver/driver.hpp"      // kinematech::Driver
#include "config/ctrl_enum.hpp"   // MotionControlType, TorqueControlType

namespace kinematech {

/// Update the driver's voltage limit and propagate it through the limits struct.
inline void KT_setVoltageLimit(Driver& d, float vlim) {
    auto lim = d.limits();
    lim.voltage_limit = vlim;
    d.setLimits(lim);
}

/// Update the driver's velocity limit.
inline void KT_setVelocityLimit(Driver& d, float wlim) {
    auto lim = d.limits();
    lim.velocity_limit = wlim;
    d.setLimits(lim);
}

/// Convenience wrapper to set the driver's target command.
inline void KT_setTarget(Driver& d, float target) {
    d.setTarget(target);
}

/// Select the motion control mode (Torque, Velocity, Angle).
inline void KT_setController(Driver& d, MotionControlType m) {
    auto cc = d.ctrl();
    cc.motion_ctrl = m;
    d.setController(cc);
}

/// Select how torque is generated (Voltage or Current loop).
inline void KT_setTorqueController(Driver& d, TorqueControlType t) {
    auto cc = d.ctrl();
    cc.torque_ctrl = t;
    d.setController(cc);
}

} // namespace kinematech

#endif // KINEMATECH_CONFIG_HELPERS_HPP

#endif /* INC_CONFIG_KINEMATECH_HELPERS_HPP_ */
