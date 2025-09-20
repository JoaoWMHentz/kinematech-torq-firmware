/*
 * ctrl_enum.hpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Enumerations and lightweight structs describing the control modes
 *  supported by the ESC firmware. They mirror the SimpleFOC naming but
 *  are kept minimal to avoid pulling large dependencies.
 */

#ifndef INC_CONFIG_CTRL_ENUM_HPP_
#define INC_CONFIG_CTRL_ENUM_HPP_

#pragma once
#ifndef KINEMATECH_CONFIG_CTRL_ENUM_HPP
#define KINEMATECH_CONFIG_CTRL_ENUM_HPP

#include <cstdint>

namespace kinematech {

/// Type of motion control loop that interprets the driver's target value.
enum class MotionControlType : uint8_t {
    Torque = 0, ///< target = torque-producing quantity (Vq or Iq)
    Velocity,   ///< target = ω_mech [rad/s]
    Angle       ///< target = θ_mech [rad]
};

/// Strategy used to generate torque.
enum class TorqueControlType : uint8_t {
    Voltage = 0, ///< torque via Vq (open-loop)
    Current      ///< torque via Iq (closed-loop FOC)
};

/// Global limits applied by the driver implementation.
struct LimitsCfg {
    float voltage_limit { 0.f };  ///< Maximum stator voltage [V]
    float current_limit { 0.f };  ///< Maximum phase current [A]
    float velocity_limit { 0.f }; ///< Maximum mechanical velocity [rad/s]
};

/// Control configuration describing motion + torque loops.
struct ControllerCfg {
    MotionControlType motion_ctrl { MotionControlType::Torque };
    TorqueControlType torque_ctrl { TorqueControlType::Voltage };
};

} // namespace kinematech

#endif // KINEMATECH_CONFIG_CTRL_ENUM_HPP

#endif /* INC_CONFIG_CTRL_ENUM_HPP_ */
