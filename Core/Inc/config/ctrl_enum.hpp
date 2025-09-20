/*
 * ctrl_enum.h
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_CONFIG_CTRL_ENUM_HPP_
#define INC_CONFIG_CTRL_ENUM_HPP_

#pragma once
#ifndef KINEMATECH_CONFIG_CTRL_ENUM_HPP
#define KINEMATECH_CONFIG_CTRL_ENUM_HPP

#include <cstdint>

namespace kinematech {

enum class MotionControlType : uint8_t {
	Torque = 0, // target = torque (Vq or Iq)
	Velocity, // target = ω_mech [rad/s]
	Angle // target = θ_mech [rad]
};

enum class TorqueControlType : uint8_t {
	Voltage = 0, // torque via Vq (open-loop)
	Current // torque via Iq (FOC closed-loop)
};

struct LimitsCfg {
	float voltage_limit { 0.f }; // [V]
	float current_limit { 0.f }; // [A]
	float velocity_limit { 0.f }; // [rad/s]
};

struct ControllerCfg {
	MotionControlType motion_ctrl { MotionControlType::Torque };
	TorqueControlType torque_ctrl { TorqueControlType::Voltage };
};

} // namespace kinematech

#endif // KINEMATECH_CONFIG_CTRL_ENUM_HPP

#endif /* INC_CONFIG_CTRL_ENUM_H_ */
