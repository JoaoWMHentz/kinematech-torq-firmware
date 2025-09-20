/*
 * driver_openloop.hpp
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_DRIVER_DRIVER_OPENLOOP_HPP_
#define INC_DRIVER_DRIVER_OPENLOOP_HPP_

#pragma once
#include "driver.hpp"
#include "math/svpwm.h"
#include "definitions.h"  // VBUS, SVPWM_LIMIT_K, OL defaults
#include <cstdint>

namespace kinematech {

class OpenLoopDriver final : public Driver {
public:
	explicit OpenLoopDriver(TIM_HandleTypeDef *tim) :
			htim_(tim) {
	}

	int init(float vbus, float loop_hz) override {
		vbus_ = vbus;
		dt_ = 1.0f / loop_hz;
		period_ = __HAL_TIM_GET_AUTORELOAD(htim_);
		theta_elec_ = 0.f;
		w_elec_ = 2.0f * 3.14159265358979323846f * OL_FREQ_ELEC_HZ; // default
		uq_set_ = OL_UQ_V;

		limits_.voltage_limit = SVPWM_LIMIT_K * vbus_;
		return 0;
	}

	void setLimits(const LimitsCfg &lim) override {
		Driver::setLimits(lim);
		v_limit_cache_ = limits_.voltage_limit;
	}

	void setController(const ControllerCfg &cc) override {
		Driver::setController(cc);
		// In open-loop only Torque/Voltage makes sense; others ignored.
	}

	void setTarget(float target) override {
		Driver::setTarget(target); // interpreted as Vq [V]
	}

	void step() override {
		// Integrate electrical angle (open-loop)
		theta_elec_ += w_elec_ * dt_;
		if (theta_elec_ > TWO_PI)
			theta_elec_ -= TWO_PI;
		if (theta_elec_ < 0.f)
			theta_elec_ += TWO_PI;

		const float Ud = 0.f;
		float Uq = target_;          // target = Vq [V] in open-loop
		if (Uq < 0.f)
			Uq = 0.f;
		if (Uq > v_limit_cache_)
			Uq = v_limit_cache_;

		svpwm(Ud, Uq, theta_elec_, v_limit_cache_, vbus_, period_, htim_);
	}

	// Convenience setters similar to SimpleFOC:
	inline void setElectricalSpeed(float w_elec_rad_s) {
		w_elec_ = w_elec_rad_s;
	}
	inline void setUq(float uq) {
		uq_set_ = uq;
		setTarget(uq);
	}

private:
	TIM_HandleTypeDef *htim_ { nullptr };
	uint32_t period_ { 0 };
	float theta_elec_ { 0.f };
	float w_elec_ { 0.f };
	float uq_set_ { 0.f };
	float v_limit_cache_ { 0.f };
};

} // namespace torq

#endif /* INC_DRIVER_DRIVER_OPENLOOP_HPP_ */
