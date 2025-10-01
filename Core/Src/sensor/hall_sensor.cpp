/*
 * hall_sensor.cpp
 *  Created on: Sep 21, 2025
 *      Author: Firmware Team
 */

#include "sensor/hall_sensor.hpp"

#include "definitions.h"
#include "utils/math_utils.hpp"
#include "utils/utils.hpp"

extern "C" {
extern volatile uint32_t uwTick;
}

namespace kinematech {

// Static instance used by ISR for dispatch
HallSensor *HallSensor::s_instance_ = nullptr;

// TIM8 handle provided by CubeMX (defined in Core/Src/main.c)
extern "C" {
extern TIM_HandleTypeDef htim8;
}

HallSensor::HallSensor(const Pin &hall_a, const Pin &hall_b, const Pin &hall_c,
                       uint8_t pole_pairs)
    : pins_{hall_a, hall_b, hall_c}, pole_pairs_(pole_pairs) {}

int HallSensor::init(float sample_hz) {
  if (pole_pairs_ == 0u) {
    return -1;
  }

  sample_period_ = (sample_hz > 0.f) ? (1.0f / sample_hz) : 0.f;
  mechanical_step_ = HALL_ELECTRICAL_STEP_RAD / static_cast<float>(pole_pairs_);
  stale_timeout_s_ = HALL_DEFAULT_STALE_S;
  if (sample_period_ > 0.f) {
    const float candidate = 5.0f * sample_period_;
    if (candidate > stale_timeout_s_) {
      stale_timeout_s_ = candidate;
    }
  }

  last_sector_ = -1;
  mechanical_angle_wrapped_ = 0.f;
  mechanical_angle_unwrapped_ = 0.f;
  mech_velocity_ = 0.f;
  last_transition_tick_ms_ = HAL_GetTick();
  predicted_angle_unwrapped_ = 0.f;
  last_transition_dt_s_ = 0.f;

  // Bind TIM8 and compute timer tick period (seconds per tick)
  tim_ = &htim8;
  const float timer_clk_hz = utils::timerInputClockHz(tim_);
  const uint32_t psc = (tim_ ? tim_->Init.Prescaler : 0u);
  if (timer_clk_hz > 0.0f) {
    const float counter_clk = timer_clk_hz / static_cast<float>(psc + 1u);
    tick_period_s_ = (counter_clk > 0.0f) ? (1.0f / counter_clk) : 0.0f;
  } else {
    tick_period_s_ = 0.0f;
  }
  last_capture_ticks_ = 0u;

  if (HALL_MAX_REASONABLE_OMEGA > 0.f) {
    min_transition_dt_s_ = mechanical_step_ / HALL_MAX_REASONABLE_OMEGA;
  } else {
    min_transition_dt_s_ = 0.f;
  }

  if (tick_period_s_ > 0.f && min_transition_dt_s_ > 0.f) {
    const float ticks = min_transition_dt_s_ / tick_period_s_;
    min_transition_ticks_ = (ticks > 1.f)
                               ? static_cast<uint32_t>(ticks + 0.999f)
                               : 1u;
    min_transition_dt_s_ = static_cast<float>(min_transition_ticks_) * tick_period_s_;
  } else {
    min_transition_ticks_ = 1u;
    if (tick_period_s_ > 0.f) {
      min_transition_dt_s_ = tick_period_s_;
    }
  }

  if (min_transition_dt_s_ <= 0.f) {
    min_transition_dt_s_ = (tick_period_s_ > 0.f) ? tick_period_s_ : HALL_MIN_DT_S;
  }

  // Latch initial state from GPIOs so angle/sector are seeded
  last_state_ = readState();
  if (last_state_ < state_table_.size()) {
    last_sector_ = state_table_[last_state_];
    if (last_sector_ >= 0) {
      mechanical_angle_wrapped_ =
          static_cast<float>(last_sector_) * mechanical_step_;
      mechanical_angle_unwrapped_ = mechanical_angle_wrapped_;
      predicted_angle_unwrapped_ = mechanical_angle_unwrapped_;
    }
  }

  // Expose this instance for ISR and start TIM8 Hall sensor in interrupt mode
  s_instance_ = this;
  if (HAL_TIMEx_HallSensor_Start_IT(tim_) != HAL_OK) {
    return -4; // failed to start hardware
  }

  return 0;
}

int HallSensor::update() {
  // Hardware-based: only handle stale detection; transitions are handled in ISR
  const uint32_t now_ms = HAL_GetTick();
  const uint32_t elapsed_ms = now_ms - last_transition_tick_ms_;
  const float dt_since = static_cast<float>(elapsed_ms) * 0.001f;

  const float current_velocity = mech_velocity_;
  const float abs_velocity = (current_velocity >= 0.f) ? current_velocity : -current_velocity;

  float dynamic_timeout = stale_timeout_s_;
  if (last_transition_dt_s_ > HALL_MIN_DT_S) {
    const float candidate = last_transition_dt_s_ * HALL_GUARD_MULTIPLIER;
    if (candidate < dynamic_timeout) {
      dynamic_timeout = candidate;
    }
  }

  const bool stale = (dynamic_timeout > 0.f) && (dt_since > dynamic_timeout);
  if (stale) {
    mech_velocity_ = 0.f;
  }

  // Refresh last_state_ (optional) to aid diagnostics
  last_state_ = readState();

  float predicted = mechanical_angle_unwrapped_;
  if (!stale && abs_velocity > 0.f && mechanical_step_ > 0.f) {
    const float max_offset = mechanical_step_ * HALL_MAX_ADVANCE_FACTOR;
    float delta = current_velocity * dt_since;
    if (max_offset > 0.f) {
      delta = utils::clamp(delta, -max_offset, max_offset);
    }
    predicted += delta;
  }

  predicted_angle_unwrapped_ = predicted;
  return 0;
}

int HallSensor::getAngle(float &theta_mech) {
  theta_mech = utils::wrap_angle(predicted_angle_unwrapped_);
  return 0;
}

int HallSensor::getVelocity(float &w_mech) {
  w_mech = mech_velocity_;
  return 0;
}

void HallSensor::setStateTable(const std::array<int8_t, 8> &table) {
  state_table_ = table;
}

uint8_t HallSensor::readState() const {
  const uint8_t hall_a =
      ((pins_[0].port->IDR & pins_[0].pin) != 0u) ? 1u : 0u;
  const uint8_t hall_b =
      ((pins_[1].port->IDR & pins_[1].pin) != 0u) ? 1u : 0u;
  const uint8_t hall_c =
      ((pins_[2].port->IDR & pins_[2].pin) != 0u) ? 1u : 0u;

  return static_cast<uint8_t>(hall_a | (hall_b << 1) | (hall_c << 2));
}
HallSensor *HallSensor::instance() { return s_instance_; }

// -----------------------------------------------------------------------------
// ISR path: called on TIM8 CH1 capture (hall transition edge)
void HallSensor::onTimerEdgeIsr(uint32_t capture_ticks) {
  const uint8_t state = readState();
  last_state_ = state;
  if (state >= state_table_.size()) {
    return;
  }

  const int sector = state_table_[state];
  if (sector < 0) {
    return; // invalid combination
  }

  float dt_s = min_transition_dt_s_;
  if (tick_period_s_ > 0.f) {
    const float dt_candidate = static_cast<float>(capture_ticks) * tick_period_s_;
    if (dt_candidate > HALL_MIN_DT_S) {
      dt_s = dt_candidate;
    } else if (sample_period_ > HALL_MIN_DT_S) {
      dt_s = sample_period_;
    }
  } else if (sample_period_ > 0.f) {
    dt_s = sample_period_;
  }
  last_capture_ticks_ = capture_ticks;

  if (last_sector_ < 0) {
    last_sector_ = sector;
    mechanical_angle_wrapped_ = static_cast<float>(sector) * mechanical_step_;
    mechanical_angle_unwrapped_ = mechanical_angle_wrapped_;
    mech_velocity_ = 0.f;
    last_transition_tick_ms_ = uwTick;
    last_transition_dt_s_ = dt_s;
    predicted_angle_unwrapped_ = mechanical_angle_unwrapped_;
    return;
  }

  if (sector == last_sector_) {
    return; // duplicate edge without sector change
  }

  int diff = sector - last_sector_;
  if (diff > 3) {
    diff -= 6;
  } else if (diff < -3) {
    diff += 6;
  }
  if (diff == 0) {
    return;
  }

  const float delta_mech = static_cast<float>(diff) * mechanical_step_;
  mechanical_angle_unwrapped_ += delta_mech;
  mechanical_angle_wrapped_ = utils::wrap_angle(mechanical_angle_wrapped_ + delta_mech);

  float omega = (dt_s > HALL_MIN_DT_S) ? (delta_mech / dt_s) : 0.f;
  if (HALL_MAX_REASONABLE_OMEGA > 0.f) {
    omega = utils::clamp(omega, -HALL_MAX_REASONABLE_OMEGA, HALL_MAX_REASONABLE_OMEGA);
  }
  mech_velocity_ = omega;
  last_transition_tick_ms_ = uwTick;
  last_transition_dt_s_ = dt_s;
  last_sector_ = sector;
  predicted_angle_unwrapped_ = mechanical_angle_unwrapped_;
}

} // namespace kinematech

// HAL captures callback (C linkage) — dispatch to HallSensor instance
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM8 && kinematech::HallSensor::instance() != nullptr) {
    const uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    kinematech::HallSensor::instance()->onTimerEdgeIsr(cap);
  }
}
