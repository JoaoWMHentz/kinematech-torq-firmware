/*
 * driver.h
 *
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 */

#ifndef INC_DRIVER_DRIVER_H_
#define INC_DRIVER_DRIVER_H_

#pragma once

#include <cstdint>
#include "config/ctrl_enum.hpp"

namespace kinematech {

class Motor;
// fwd decl (defined in motor/motor.hpp)
class Sensor;
// fwd decl (defined in sensor/sensor.hpp)

class Driver {
public:
	virtual ~Driver() = default;

	// Wiring
	virtual void attachMotor(Motor *m) {
		motor_ = m;
	}
	virtual void attachSensor(Sensor *s) {
		sensor_ = s;
	}

	// Config
	virtual int init(float vbus, float loop_hz) = 0;
	virtual void setLimits(const LimitsCfg &lim) {
		limits_ = lim;
	}
	virtual void setController(const ControllerCfg &cc) {
		ctrl_ = cc;
	}

	// Command
	virtual void setTarget(float target) {
		target_ = target;
	}

	// ISR step (called at PWM rate)
	virtual void step() = 0;

	// Accessors
	inline float vbus() const {
		return vbus_;
	}
	inline float dt() const {
		return dt_;
	}
	inline const LimitsCfg& limits() const {
		return limits_;
	}
	inline const ControllerCfg& ctrl() const {
		return ctrl_;
	}

protected:
	Motor *motor_ { nullptr };
	Sensor *sensor_ { nullptr };

	LimitsCfg limits_ { };
	ControllerCfg ctrl_ { };

	float target_ { 0.f };
	float vbus_ { 0.f };
	float dt_ { 0.f };
};

} // namespace kinematech

#endif /* INC_DRIVER_DRIVER_H_ */
