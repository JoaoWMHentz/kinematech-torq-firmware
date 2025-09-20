/*
 * motor.h
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_MOTOR_MOTOR_HPP_
#define INC_MOTOR_MOTOR_HPP_

#pragma once
#include <cstdint>

namespace kinematech {

struct MotorCfg {
	uint8_t pole_pairs { 1 };
	float phase_resistance { 0.f };
	float phase_inductance { 0.f };
	float kv_rpm_per_v { 0.f };
	float kt_Nm_per_A { 0.f };
};

class Motor {
public:
	explicit Motor(const MotorCfg &cfg) :
			cfg_(cfg) {
	}

	inline uint8_t polePairs() const {
		return cfg_.pole_pairs;
	}
	inline const MotorCfg& cfg() const {
		return cfg_;
	}

	// Cached state (optional):
	float electrical_angle { 0.f };
	float mechanical_angle { 0.f };
	float mech_velocity { 0.f };

	static inline float elecFromMech(const Motor &m, float theta_mech) {
		return theta_mech * m.cfg_.pole_pairs;
	}

private:
	MotorCfg cfg_;
};

} // namespace torq

#endif /* INC_MOTOR_MOTOR_H_ */
