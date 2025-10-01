
/*
 * closedloop_driver.cpp
 *  Created on: Nov 7, 2025
 *      Author: Firmware Team
 *
 *  Closed-loop field-oriented controller inspired by SimpleFOC. Implements the
 *  typical cascaded PI structure (position → velocity → torque) and produces
 *  SVPWM duty cycles that are applied to TIM1.
 */

#include "driver/driver_closedloop.hpp"

#include <cmath>

#if defined(__has_include)
#if __has_include("arm_math.h")
#if defined(__CORTEX_M)
#if (__CORTEX_M == 0)
#ifndef ARM_MATH_CM0
#define ARM_MATH_CM0
#endif
#elif (__CORTEX_M == 3)
#ifndef ARM_MATH_CM3
#define ARM_MATH_CM3
#endif
#elif (__CORTEX_M == 4)
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#elif (__CORTEX_M == 7)
#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif
#elif (__CORTEX_M == 33)
#ifndef ARM_MATH_CM33
#define ARM_MATH_CM33
#endif
#endif
#endif
extern "C" {
#include "arm_math.h"
}
#define KINEMATECH_HAS_ARM_MATH 1
#endif
#endif

#ifndef KINEMATECH_HAS_ARM_MATH
#define KINEMATECH_HAS_ARM_MATH 0
#endif

#include "definitions.h"
#include "math/svpwm.h"
#include "motor/motor.hpp"
#include "sensor/sensor.hpp"

namespace kinematech {
namespace {
constexpr float kTwoOverThree = 0.6666666667f;   // 2/3, Park/Clarke scaling
constexpr float kOneOverSqrt3 = 0.5773502692f;   // 1/√3, alpha-beta scaling

inline void sincosFast(float angle, float& s, float& c) {
#if KINEMATECH_HAS_ARM_MATH
    arm_sin_cos_f32(angle, &s, &c);
#else
    s = std::sin(angle);
    c = std::cos(angle);
#endif
}
} // namespace

	ClosedLoopDriver::ClosedLoopDriver(TIM_HandleTypeDef* tim)
    : htim_(tim) {
    refreshControlDispatch();
}

int ClosedLoopDriver::init(float vbus, float loop_hz) {
    if (loop_hz <= 0.f || !htim_) {
        return -1; // invalid initialization parameters
    }

    // Cache bus voltage and loop period (executed at PWM rate).
    vbus_ = vbus;
    dt_ = 1.0f / loop_hz;

#if defined(__HAL_TIM_GET_AUTORELOAD)
    period_ = __HAL_TIM_GET_AUTORELOAD(htim_);
#else
    period_ = (htim_ ? htim_->Init.Period : 0u);
#endif

    // Reset controllers/state so previous runs do not leak.
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

    // Provide SimpleFOC-like default gains if user has not tuned them yet.
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

    setVelocityFilterCutoff(200.0f); // light smoothing for hall velocity

    theta_mech_prev_raw_ = 0.f;
    theta_mech_prev_unwrapped_ = 0.f;
    theta_mech_raw_ = 0.f;
    theta_mech_unwrapped_ = 0.f;
    theta_mech_delta_ = 0.f;
    theta_elec_ = 0.f;
    sensor_ready_ = false;
    velocity_meas_ = 0.f;
    velocity_filtered_ = 0.f;
    velocity_filter_alpha_clamped_ = 1.0f;
    torque_target_ = 0.f;
    iq_target_ = 0.f;
    voltage_cmd_ = {};
    current_meas_ = {};
    current_valid_ = false;
    uq_prev_ = 0.f;
    stall_timer_ = 0.f;

    refreshControlDispatch();

    // Seed sensor state so the first control step starts with a valid angle.
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
    has_velocity_limit_ = (limits_.velocity_limit > 0.f);
    velocity_pi_.limit = (limits_.voltage_limit > 0.f) ? limits_.voltage_limit : velocity_pi_.limit;
    current_pi_.limit = (limits_.voltage_limit > 0.f) ? limits_.voltage_limit : current_pi_.limit;
    position_pi_.limit = (limits_.velocity_limit > 0.f) ? limits_.velocity_limit : position_pi_.limit;

    if (limits_.velocity_limit > 0.f) {
        const float thresh = 0.05f * limits_.velocity_limit;
        if (thresh > 0.5f) {
            stall_velocity_threshold_ = thresh;
        }
    }
}

void ClosedLoopDriver::setController(const ControllerCfg& cc) {
    Driver::setController(cc);
    refreshControlDispatch();
}

void ClosedLoopDriver::setTarget(float target) {
    last_target_ = target;

    // Clamp user target according to active control mode and limits.
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
    const float active_v_limit = computeVoltageLimit();
    torqueLoop(torque_ref, active_v_limit);
    monitorStall(active_v_limit);
    applyFOC(active_v_limit);
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
        velocity_filter_alpha_ = 1.0f; // instantaneous update (no filtering)
        velocity_filter_alpha_clamped_ = 1.0f;
        return;
    }
    const float tau = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff_hz);
    velocity_filter_alpha_ = dt_ / (dt_ + tau);
    velocity_filter_alpha_clamped_ = clamp(velocity_filter_alpha_, 0.f, 1.f);
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
    const bool has_limit = (limit > 0.f);
    const float proportional = kp * error;

    // Anti-windup: only integrate when not saturated or when error drives back.
    bool prevent_int = false;
    if (has_limit) {
        if ((integral >= limit && error > 0.f) ||
            (integral <= -limit && error < 0.f)) {
            prevent_int = true;
        }
    }

    float new_integral = integral;
    if (!prevent_int) {
        new_integral += ki * error * dt;
    }

    if (has_limit) {
        if (new_integral > limit) {
            new_integral = limit;
        } else if (new_integral < -limit) {
            new_integral = -limit;
        }
    }

    float out = proportional + new_integral;
    if (has_limit) {
        if (out > limit) {
            out = limit;
        } else if (out < -limit) {
            out = -limit;
        }
    }

    integral = new_integral;
    return out;
}

void ClosedLoopDriver::PIController::reset() {
    integral = 0.f;
}

void ClosedLoopDriver::PIController::dampen(float factor) {
    if (factor < 0.f) {
        factor = 0.f;
    } else if (factor > 1.f) {
        factor = 1.f;
    }
    integral *= factor;
}

void ClosedLoopDriver::sampleSensor() {
    if (!sensor_) {
        return;
    }

    // Sensor housekeeping runs in ESC_Main_Loop to keep this ISR lean.

    // Capture mechanical angle and unwrap it so the controller stays continuous.
    float theta = theta_mech_raw_;
    if (sensor_->getAngle(theta) == 0) {
        theta_mech_raw_ = theta;
        theta_mech_unwrapped_ = unwrapAngle(theta);
    }

    // Prefer sensor-provided velocity; fall back to finite difference.
    float vel = 0.f;
    bool vel_ok = (sensor_->getVelocity(vel) == 0);
    if (!vel_ok && sensor_ready_) {
        vel = theta_mech_delta_ / dt_;
        vel_ok = true;
    }

    if (vel_ok) {
        velocity_meas_ = vel;
        velocity_filtered_ += velocity_filter_alpha_clamped_ * (velocity_meas_ - velocity_filtered_);
    }

    if (motor_) {
        motor_->mechanical_angle = theta_mech_raw_;
        motor_->mech_velocity = velocity_filtered_;
    }

    // Convert mechanical angle into electrical and store it for SVPWM.
    if (motor_) {
        const float elec = static_cast<float>(sensor_dir_) * Motor::elecFromMech(*motor_, theta_mech_unwrapped_) + zero_elec_offset_;
        theta_elec_ = wrapAngle(elec);
        sincosFast(theta_elec_, sin_theta_elec_, cos_theta_elec_);
        motor_->electrical_angle = theta_elec_;
    } else {
        theta_elec_ = wrapAngle(static_cast<float>(sensor_dir_) * theta_mech_unwrapped_ + zero_elec_offset_);
        sincosFast(theta_elec_, sin_theta_elec_, cos_theta_elec_);
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
    return (this->*setpoint_fn_)();
}

float ClosedLoopDriver::velocityLoop(float velocity_target) {
    const float error = velocity_target - velocity_filtered_;
    return velocity_pi_.apply(error, dt_);
}

float ClosedLoopDriver::positionLoop(float position_target) {
    const float error = position_target - theta_mech_unwrapped_;
    return position_pi_.apply(error, dt_);
}

float ClosedLoopDriver::torqueLoop(float torque_target, float v_limit) {
    torque_target_ = torque_target;
    return (this->*torque_loop_fn_)(torque_target, v_limit);
}

void ClosedLoopDriver::applyFOC(float active_v_limit) {
    if (!htim_ || period_ == 0u) {
        return;
    }

    const float Ud = voltage_cmd_.d;
    const float Uq = voltage_cmd_.q;
    const float vbus = (vbus_ != 0.f) ? vbus_ : VBUS_V;
    float v_limit = active_v_limit;
    if (v_limit <= 0.f) {
        v_limit = (limits_.voltage_limit > 0.f) ? limits_.voltage_limit : (SVPWM_LIMIT_K * vbus);
    }

    svpwm(Ud, Uq, theta_elec_, v_limit, vbus, period_, htim_);
}

void ClosedLoopDriver::updateCurrentDQ(const PhaseCurrents& iabc) {
    // Clarke transform: convert three-phase currents into alpha/beta frame.
    const float i_alpha = kTwoOverThree * (iabc.ia - 0.5f * (iabc.ib + iabc.ic));
    const float i_beta = kTwoOverThree * kOneOverSqrt3 * (iabc.ib - iabc.ic);

    // Park transform: rotate alpha/beta into dq aligned with electrical angle.
    const float s = sin_theta_elec_;
    const float c = cos_theta_elec_;

    current_meas_.d = c * i_alpha + s * i_beta;
    current_meas_.q = -s * i_alpha + c * i_beta;
    current_valid_ = true;
}

float ClosedLoopDriver::computeVoltageLimit() const {
    if (v_limit_cache_ > 0.f) {
        return v_limit_cache_;
    }
    if (limits_.voltage_limit > 0.f) {
        return limits_.voltage_limit;
    }
    const float vbus = (vbus_ != 0.f) ? vbus_ : VBUS_V;
    return SVPWM_LIMIT_K * vbus;
}

float ClosedLoopDriver::applyVoltageSlew(float requested_q) {
    if (dt_ <= 0.f || uq_slew_rate_ <= 0.f) {
        uq_prev_ = requested_q;
        return requested_q;
    }

    const float max_delta = uq_slew_rate_ * dt_;
    if (max_delta <= 0.f) {
        uq_prev_ = requested_q;
        return requested_q;
    }

    const float delta = clamp(requested_q - uq_prev_, -max_delta, max_delta);
    uq_prev_ += delta;
    return uq_prev_;
}

void ClosedLoopDriver::monitorStall(float v_limit) {
    if (v_limit <= 0.f) {
        stall_timer_ = 0.f;
        return;
    }

    const float abs_voltage = std::abs(voltage_cmd_.q);
    const float abs_velocity = std::abs(velocity_filtered_);
    const bool saturating = abs_voltage > (0.85f * v_limit);

    if (saturating && abs_velocity < stall_velocity_threshold_) {
        stall_timer_ += dt_;
        if (stall_timer_ >= stall_timeout_s_) {
            velocity_pi_.dampen(stall_decay_factor_);
            current_pi_.dampen(stall_decay_factor_);
            uq_prev_ *= stall_decay_factor_;
            voltage_cmd_.q = uq_prev_;
            stall_timer_ = 0.f;
        }
    } else {
        stall_timer_ = 0.f;
    }
}

void ClosedLoopDriver::refreshControlDispatch() {
    switch (ctrl_.motion_ctrl) {
    case MotionControlType::Torque:
        setpoint_fn_ = &ClosedLoopDriver::computeSetpointTorqueMode;
        break;
    case MotionControlType::Velocity:
        setpoint_fn_ = &ClosedLoopDriver::computeSetpointVelocityMode;
        break;
    case MotionControlType::Angle:
        setpoint_fn_ = &ClosedLoopDriver::computeSetpointAngleMode;
        break;
    default:
        setpoint_fn_ = &ClosedLoopDriver::computeSetpointFallback;
        break;
    }

    torque_loop_fn_ = (ctrl_.torque_ctrl == TorqueControlType::Current)
        ? &ClosedLoopDriver::torqueLoopCurrentMode
        : &ClosedLoopDriver::torqueLoopVoltageMode;
}

float ClosedLoopDriver::computeSetpointTorqueMode() {
    return last_target_;
}

float ClosedLoopDriver::computeSetpointVelocityMode() {
    float vel_target = last_target_;
    if (has_velocity_limit_) {
        vel_target = clamp(vel_target, -limits_.velocity_limit, limits_.velocity_limit);
    }
    return velocityLoop(vel_target);
}

float ClosedLoopDriver::computeSetpointAngleMode() {
    float vel_target = positionLoop(last_target_);
    if (has_velocity_limit_) {
        vel_target = clamp(vel_target, -limits_.velocity_limit, limits_.velocity_limit);
    }
    return velocityLoop(vel_target);
}

float ClosedLoopDriver::computeSetpointFallback() {
    return last_target_;
}

float ClosedLoopDriver::torqueLoopVoltageMode(float torque_target, float v_limit) {
    voltage_cmd_.d = 0.f;
    const bool has_limit = (v_limit > 0.f);
    const float uq_limited = has_limit ? clamp(torque_target, -v_limit, v_limit) : torque_target;
    voltage_cmd_.q = applyVoltageSlew(uq_limited);
    return voltage_cmd_.q;
}

float ClosedLoopDriver::torqueLoopCurrentMode(float torque_target, float v_limit) {
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
    const bool has_limit = (v_limit > 0.f);
    const float uq_limited = has_limit ? clamp(uq_cmd, -v_limit, v_limit) : uq_cmd;
    voltage_cmd_.q = applyVoltageSlew(uq_limited);
    return voltage_cmd_.q;
}

} // namespace kinematech
