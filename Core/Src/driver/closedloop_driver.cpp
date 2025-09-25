/*
 * closedloop_driver.cpp
 *  Created on: Nov 7, 2025
 *      Author: Firmware Team
 */

#include "driver/driver_closedloop.hpp"

#include <cmath>

#include "definitions.h"
#include "math/svpwm.h"
#include "motor/motor.hpp"
#include "sensor/sensor.hpp"

namespace kinematech {
namespace {
constexpr float kTwoOverThree = 0.6666666667f;
constexpr float kOneOverSqrt3 = 0.5773502692f;
}

ClosedLoopDriver::ClosedLoopDriver(TIM_HandleTypeDef* tim)
    : htim_(tim) {}

int ClosedLoopDriver::init(float vbus, float loop_hz) {
    if (loop_hz <= 0.f || !htim_) {
        return -1;
    }

    vbus_ = vbus;
    dt_ = 1.0f / loop_hz;

#if defined(__HAL_TIM_GET_AUTORELOAD)
    period_ = __HAL_TIM_GET_AUTORELOAD(htim_);
#else
    period_ = (htim_ ? htim_->Init.Period : 0u);
#endif

    velocity_pi_.reset();
    position_pi_.reset();
    current_pi_.reset();

    const float default_vlimit = (limits_.voltage_limit > 0.f) ? limits_.voltage_limit : (SVPWM_LIMIT_K * vbus_);
    const float default_vel_limit = (limits_.velocity_limit > 0.f) ? limits_.velocity_limit : 0.f;

    if (velocity_pi_.limit <= 0.f) {
        velocity_pi_.limit = default_vlimit;
    }
    if (current_pi_.limit <= 0.f) {
        current_pi_.limit = default_vlimit;
    }
    if (position_pi_.limit <= 0.f && default_vel_limit > 0.f) {
        position_pi_.limit = default_vel_limit;
    }

    if (velocity_pi_.kp == 0.f && velocity_pi_.ki == 0.f) {
        velocity_pi_.kp = 0.3f;
        velocity_pi_.ki = 10.0f;
    }
    if (position_pi_.kp == 0.f && position_pi_.ki == 0.f) {
        position_pi_.kp = 5.0f;
        position_pi_.ki = 0.0f;
    }
    if (current_pi_.kp == 0.f && current_pi_.ki == 0.f) {
        current_pi_.kp = 1.0f;
        current_pi_.ki = 0.0f;
    }

    setVelocityFilterCutoff(200.0f);

    theta_mech_prev_raw_ = 0.f;
    theta_mech_prev_unwrapped_ = 0.f;
    theta_mech_raw_ = 0.f;
    theta_mech_unwrapped_ = 0.f;
    theta_mech_delta_ = 0.f;
    theta_elec_ = 0.f;
    sensor_ready_ = false;
    velocity_meas_ = 0.f;
    velocity_filtered_ = 0.f;
    torque_target_ = 0.f;
    iq_target_ = 0.f;
    voltage_cmd_ = {};
    current_meas_ = {};
    current_valid_ = false;

    if (sensor_) {
        if (sensor_->init(loop_hz) == 0) {
            sensor_->update();
            float seed = 0.f;
            if (sensor_->getAngle(seed) == 0) {
                unwrapAngle(seed);
            }
        }
    }

    return 0;
}

void ClosedLoopDriver::setLimits(const LimitsCfg& lim) {
    Driver::setLimits(lim);
    v_limit_cache_ = limits_.voltage_limit;
    velocity_pi_.limit = (limits_.voltage_limit > 0.f) ? limits_.voltage_limit : velocity_pi_.limit;
    current_pi_.limit = (limits_.voltage_limit > 0.f) ? limits_.voltage_limit : current_pi_.limit;
    position_pi_.limit = (limits_.velocity_limit > 0.f) ? limits_.velocity_limit : position_pi_.limit;
}

void ClosedLoopDriver::setController(const ControllerCfg& cc) {
    Driver::setController(cc);
}

void ClosedLoopDriver::setTarget(float target) {
    last_target_ = target;

    if (ctrl_.motion_ctrl == MotionControlType::Torque &&
        ctrl_.torque_ctrl == TorqueControlType::Voltage && v_limit_cache_ > 0.f) {
        last_target_ = clamp(last_target_, -v_limit_cache_, v_limit_cache_);
    }

    if (ctrl_.motion_ctrl == MotionControlType::Velocity && limits_.velocity_limit > 0.f) {
        last_target_ = clamp(last_target_, -limits_.velocity_limit, limits_.velocity_limit);
    }

    Driver::setTarget(last_target_);
}

void ClosedLoopDriver::step() {
    sampleSensor();
    const float torque_ref = computeTorqueSetpoint();
    const float uq = torqueLoop(torque_ref);
    applyFOC(uq);
}

void ClosedLoopDriver::setVelocityGains(const PIConfig& cfg) {
    velocity_pi_.kp = cfg.kp;
    velocity_pi_.ki = cfg.ki;
    velocity_pi_.limit = cfg.limit;
    velocity_pi_.reset();
}

void ClosedLoopDriver::setPositionGains(const PIConfig& cfg) {
    position_pi_.kp = cfg.kp;
    position_pi_.ki = cfg.ki;
    position_pi_.limit = cfg.limit;
    position_pi_.reset();
}

void ClosedLoopDriver::setCurrentGains(const PIConfig& cfg) {
    current_pi_.kp = cfg.kp;
    current_pi_.ki = cfg.ki;
    current_pi_.limit = cfg.limit;
    current_pi_.reset();
}

void ClosedLoopDriver::setVelocityFilterCutoff(float cutoff_hz) {
    if (cutoff_hz <= 0.f || dt_ <= 0.f) {
        velocity_filter_alpha_ = 1.0f;
        return;
    }
    const float tau = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff_hz);
    velocity_filter_alpha_ = dt_ / (dt_ + tau);
}

void ClosedLoopDriver::setSensorDirection(int8_t dir) {
    sensor_dir_ = (dir >= 0) ? 1 : -1;
}

void ClosedLoopDriver::setElectricalZero(float offset_rad) {
    zero_elec_offset_ = offset_rad;
}

void ClosedLoopDriver::feedPhaseCurrents(const PhaseCurrents& iabc) {
    updateCurrentDQ(iabc);
}

void ClosedLoopDriver::feedPhaseCurrents(float ia, float ib, float ic) {
    feedPhaseCurrents(PhaseCurrents{ia, ib, ic});
}

float ClosedLoopDriver::electricalAngle() const {
    return theta_elec_;
}

float ClosedLoopDriver::shaftAngle() const {
    return theta_mech_unwrapped_;
}

float ClosedLoopDriver::shaftVelocity() const {
    return velocity_filtered_;
}

float ClosedLoopDriver::qVoltage() const {
    return voltage_cmd_.q;
}

float ClosedLoopDriver::PIController::apply(float error, float dt) {
    integral += ki * error * dt;
    if (limit > 0.f) {
        if (integral > limit) {
            integral = limit;
        } else if (integral < -limit) {
            integral = -limit;
        }
    }
    float out = kp * error + integral;
    if (limit > 0.f) {
        if (out > limit) {
            out = limit;
        } else if (out < -limit) {
            out = -limit;
        }
    }
    return out;
}

void ClosedLoopDriver::PIController::reset() {
    integral = 0.f;
}

void ClosedLoopDriver::sampleSensor() {
    if (!sensor_) {
        return;
    }

    sensor_->update();

    float theta = theta_mech_raw_;
    if (sensor_->getAngle(theta) == 0) {
        theta_mech_raw_ = theta;
        theta_mech_unwrapped_ = unwrapAngle(theta);
    }

    float vel = 0.f;
    bool vel_ok = (sensor_->getVelocity(vel) == 0);
    if (!vel_ok && sensor_ready_) {
        vel = theta_mech_delta_ / dt_;
        vel_ok = true;
    }

    if (vel_ok) {
        velocity_meas_ = vel;
        const float alpha = clamp(velocity_filter_alpha_, 0.f, 1.f);
        velocity_filtered_ += alpha * (velocity_meas_ - velocity_filtered_);
    }

    if (motor_) {
        motor_->mechanical_angle = theta_mech_raw_;
        motor_->mech_velocity = velocity_filtered_;
    }

    if (motor_) {
        const float elec = static_cast<float>(sensor_dir_) * Motor::elecFromMech(*motor_, theta_mech_unwrapped_) + zero_elec_offset_;
        theta_elec_ = wrapAngle(elec);
        motor_->electrical_angle = theta_elec_;
    } else {
        theta_elec_ = wrapAngle(static_cast<float>(sensor_dir_) * theta_mech_unwrapped_ + zero_elec_offset_);
    }
}

float ClosedLoopDriver::unwrapAngle(float raw_angle) {
    if (!sensor_ready_) {
        sensor_ready_ = true;
        theta_mech_prev_raw_ = raw_angle;
        theta_mech_prev_unwrapped_ = raw_angle;
        theta_mech_delta_ = 0.f;
        return raw_angle;
    }

    float diff = raw_angle - theta_mech_prev_raw_;
    if (diff > static_cast<float>(M_PI)) {
        diff -= TWO_PI;
    } else if (diff < -static_cast<float>(M_PI)) {
        diff += TWO_PI;
    }

    const float unwrapped = theta_mech_prev_unwrapped_ + diff;
    theta_mech_prev_raw_ = raw_angle;
    theta_mech_prev_unwrapped_ = unwrapped;
    theta_mech_delta_ = diff;
    return unwrapped;
}

float ClosedLoopDriver::wrapAngle(float angle) const {
    while (angle >= TWO_PI) {
        angle -= TWO_PI;
    }
    while (angle < 0.f) {
        angle += TWO_PI;
    }
    return angle;
}

float ClosedLoopDriver::clamp(float value, float min_val, float max_val) const {
    return (value < min_val) ? min_val : ((value > max_val) ? max_val : value);
}

float ClosedLoopDriver::computeTorqueSetpoint() {
    switch (ctrl_.motion_ctrl) {
    case MotionControlType::Torque:
        return last_target_;
    case MotionControlType::Velocity: {
        float vel_target = last_target_;
        if (limits_.velocity_limit > 0.f) {
            vel_target = clamp(vel_target, -limits_.velocity_limit, limits_.velocity_limit);
        }
        return velocityLoop(vel_target);
    }
    case MotionControlType::Angle: {
        float vel_target = positionLoop(last_target_);
        if (limits_.velocity_limit > 0.f) {
            vel_target = clamp(vel_target, -limits_.velocity_limit, limits_.velocity_limit);
        }
        return velocityLoop(vel_target);
    }
    default:
        return last_target_;
    }
}

float ClosedLoopDriver::velocityLoop(float velocity_target) {
    const float error = velocity_target - velocity_filtered_;
    return velocity_pi_.apply(error, dt_);
}

float ClosedLoopDriver::positionLoop(float position_target) {
    const float error = position_target - theta_mech_unwrapped_;
    return position_pi_.apply(error, dt_);
}

float ClosedLoopDriver::torqueLoop(float torque_target) {
    if (ctrl_.torque_ctrl == TorqueControlType::Current) {
        iq_target_ = torque_target;
        if (limits_.current_limit > 0.f) {
            iq_target_ = clamp(iq_target_, -limits_.current_limit, limits_.current_limit);
        }

        float uq_cmd = 0.f;
        if (current_valid_) {
            const float error = iq_target_ - current_meas_.q;
            uq_cmd = current_pi_.apply(error, dt_);
        } else if (motor_ && motor_->cfg().phase_resistance > 0.f) {
            uq_cmd = iq_target_ * motor_->cfg().phase_resistance;
        } else {
            uq_cmd = iq_target_;
        }

        voltage_cmd_.d = 0.f;
        voltage_cmd_.q = (v_limit_cache_ > 0.f) ? clamp(uq_cmd, -v_limit_cache_, v_limit_cache_) : uq_cmd;
        return voltage_cmd_.q;
    }

    voltage_cmd_.d = 0.f;
    voltage_cmd_.q = (v_limit_cache_ > 0.f) ? clamp(torque_target, -v_limit_cache_, v_limit_cache_) : torque_target;
    return voltage_cmd_.q;
}

void ClosedLoopDriver::applyFOC(float uq_cmd) {
    (void)uq_cmd;
    if (!htim_ || period_ == 0u) {
        return;
    }

    const float Ud = voltage_cmd_.d;
    const float Uq = voltage_cmd_.q;
    const float vbus = (vbus_ != 0.f) ? vbus_ : VBUS_V;
    float v_limit = v_limit_cache_;
    if (v_limit <= 0.f) {
        v_limit = (limits_.voltage_limit > 0.f) ? limits_.voltage_limit : (SVPWM_LIMIT_K * vbus);
    }

    svpwm(Ud, Uq, theta_elec_, v_limit, vbus, period_, htim_);
}

void ClosedLoopDriver::updateCurrentDQ(const PhaseCurrents& iabc) {
    const float i_alpha = kTwoOverThree * (iabc.ia - 0.5f * (iabc.ib + iabc.ic));
    const float i_beta = kTwoOverThree * kOneOverSqrt3 * (iabc.ib - iabc.ic);

    const float s = std::sinf(theta_elec_);
    const float c = std::cosf(theta_elec_);

    current_meas_.d = c * i_alpha + s * i_beta;
    current_meas_.q = -s * i_alpha + c * i_beta;
    current_valid_ = true;
}

} // namespace kinematech
