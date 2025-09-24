/*
 * openloop_driver.cpp
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Open-loop motor driver using Space Vector PWM (SVPWM).
 *  Applies a commanded q-axis voltage (Vq) at a given electrical speed.
 */

#include "driver/driver_openloop.hpp"
#include "math/svpwm.h"
#include "definitions.h"
#include "sensor/sensor.hpp"
#include "motor/motor.hpp"

extern "C" {
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
}

#include <cmath>

namespace kinematech {

OpenLoopDriver::OpenLoopDriver(TIM_HandleTypeDef* tim)
: htim_(tim) {}

int OpenLoopDriver::init(float vbus, float loop_hz) {
    vbus_       = vbus;
    dt_         = 1.0f / loop_hz;

#if defined(__HAL_TIM_GET_AUTORELOAD)
    period_     = __HAL_TIM_GET_AUTORELOAD(htim_);   // PWM period from timer ARR
#else
    period_     = (htim_ ? htim_->Init.Period : 0);  // fallback
#endif

    theta_elec_ = 0.f;                               // initial angle
    w_elec_     = TWO_PI * OL_FREQ_ELEC_HZ;          // default electrical speed
    uq_set_     = OL_UQ_V;                           // default Vq command

    limits_.voltage_limit = SVPWM_LIMIT_K * vbus_;   // max usable voltage
    v_limit_cache_        = limits_.voltage_limit;
    if (sensor_) {
        if (sensor_->init(loop_hz) == 0) {
            sensor_->update();
        }
    }

    return 0;
}

void OpenLoopDriver::setLimits(const LimitsCfg& lim) {
    Driver::setLimits(lim);
    v_limit_cache_ = limits_.voltage_limit;
}

void OpenLoopDriver::setController(const ControllerCfg& cc) {
    Driver::setController(cc);
}

void OpenLoopDriver::setTarget(float target) {
    Driver::setTarget(target);
}

/**
 * @brief Core open-loop calculation step.
 *
 * - Integrates electrical angle: θ(k+1) = θ(k) + ω * Δt
 * - Wraps θ into [0, 2π)
 * - Uses target as Vq (q-axis voltage command)
 * - Clamps Vq to [0, voltage_limit]
 * - Calls SVPWM to convert (Ud=0, Uq, θ) into duty cycles
 */
void OpenLoopDriver::step() {
    if (sensor_) {
        sensor_->update();
        if (motor_) {
            float theta_mech = 0.f;
            if (sensor_->getAngle(theta_mech) == 0) {
                motor_->mechanical_angle = theta_mech;
                motor_->electrical_angle = Motor::elecFromMech(*motor_, theta_mech);
            }

            float vel_mech = 0.f;
            if (sensor_->getVelocity(vel_mech) == 0) {
                motor_->mech_velocity = vel_mech;
            }
        }
    }

    // θ(k+1) = θ(k) + ω * Δt
    theta_elec_ += w_elec_ * dt_;

    // wrap into [0, 2π)
    if (theta_elec_ > TWO_PI)  theta_elec_ -= TWO_PI;
    if (theta_elec_ < 0.f)     theta_elec_ += TWO_PI;

    const float Ud = 0.f;      // no d-axis voltage
    float Uq = target_;        // commanded Vq

    // clamp Vq into valid range
    if (Uq < 0.f)            Uq = 0.f;
    if (Uq > v_limit_cache_) Uq = v_limit_cache_;

    // update PWM outputs
    svpwm(Ud, Uq, theta_elec_, v_limit_cache_, vbus_, period_, htim_);
}

void OpenLoopDriver::setElectricalSpeed(float w_elec_rad_s) {
    w_elec_ = w_elec_rad_s;
}

void OpenLoopDriver::setUq(float uq) {
    uq_set_ = uq;
    setTarget(uq);
}

} // namespace kinematech
