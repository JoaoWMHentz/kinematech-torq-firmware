/*
 * hall_sensor.cpp
 *  Created on: Sep 21, 2025
 *      Author: Firmware Team
 */

#include "sensor/hall_sensor.hpp"

#include "definitions.h"

namespace kinematech {
namespace {
constexpr float kElectricalStep = TWO_PI / 6.0f; // 60 electrical degrees
constexpr float kMinDt = 1e-7f;
constexpr float kDefaultStale = 0.1f; // seconds before velocity decays to zero
constexpr float kMaxReasonableOmega = 2000.0f; // rad/s guard for glitches

static float timerInputClockHz(const TIM_HandleTypeDef* tim) {
  if (tim == nullptr) {
    return 0.0f;
  }

  RCC_ClkInitTypeDef clk_cfg {};
  uint32_t flash_latency = 0u;
  HAL_RCC_GetClockConfig(&clk_cfg, &flash_latency);

  const bool is_apb2_timer =
      tim->Instance == TIM1 || tim->Instance == TIM8 || tim->Instance == TIM15 ||
      tim->Instance == TIM16 || tim->Instance == TIM17;

  uint32_t pclk_hz = 0u;
  if (is_apb2_timer) {
    pclk_hz = HAL_RCC_GetPCLK2Freq();
    if (clk_cfg.APB2CLKDivider != RCC_HCLK_DIV1) {
      pclk_hz *= 2u;
    }
  } else {
    pclk_hz = HAL_RCC_GetPCLK1Freq();
    if (clk_cfg.APB1CLKDivider != RCC_HCLK_DIV1) {
      pclk_hz *= 2u;
    }
  }

  return static_cast<float>(pclk_hz);
}

} // namespace

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
  mechanical_step_ = kElectricalStep / static_cast<float>(pole_pairs_);
  stale_timeout_s_ = kDefaultStale;
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
  last_transition_time_s_ = nowSeconds();

  // Bind TIM8 and compute timer tick period (seconds per tick)
  tim_ = &htim8;
  const float timer_clk_hz = timerInputClockHz(tim_);
  const uint32_t psc = (tim_ ? tim_->Init.Prescaler : 0u);
  if (timer_clk_hz > 0.0f) {
    const float counter_clk = timer_clk_hz / static_cast<float>(psc + 1u);
    tick_period_s_ = (counter_clk > 0.0f) ? (1.0f / counter_clk) : 0.0f;
  } else {
    tick_period_s_ = 0.0f;
  }
  last_capture_ticks_ = 0u;

  if (kMaxReasonableOmega > 0.f) {
    min_transition_dt_s_ = mechanical_step_ / kMaxReasonableOmega;
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
    min_transition_dt_s_ = (tick_period_s_ > 0.f) ? tick_period_s_ : kMinDt;
  }

  // Latch initial state from GPIOs so angle/sector are seeded
  last_state_ = readState();
  if (last_state_ < state_table_.size()) {
    last_sector_ = state_table_[last_state_];
    if (last_sector_ >= 0) {
      mechanical_angle_wrapped_ =
          static_cast<float>(last_sector_) * mechanical_step_;
      mechanical_angle_unwrapped_ = mechanical_angle_wrapped_;
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
  const float now = nowSeconds();
  if ((now - last_transition_time_s_) > stale_timeout_s_) {
    mech_velocity_ = 0.f;
  }
  // Refresh last_state_ (optional) to aid diagnostics
  last_state_ = readState();
  return 0;
}

int HallSensor::getAngle(float &theta_mech) {
  theta_mech = mechanical_angle_wrapped_;
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
  uint8_t hall_a =
      HAL_GPIO_ReadPin(pins_[0].port, pins_[0].pin) == GPIO_PIN_SET ? 1u : 0u;
  uint8_t hall_b =
      HAL_GPIO_ReadPin(pins_[1].port, pins_[1].pin) == GPIO_PIN_SET ? 1u : 0u;
  uint8_t hall_c =
      HAL_GPIO_ReadPin(pins_[2].port, pins_[2].pin) == GPIO_PIN_SET ? 1u : 0u;

  return static_cast<uint8_t>(hall_a | (hall_b << 1) | (hall_c << 2));
}

float HallSensor::nowSeconds() {
  return static_cast<float>(HAL_GetTick()) / 1000.0f;
}

float HallSensor::wrapAngle(float angle) {
  if (angle >= TWO_PI) {
    angle -= TWO_PI;
  }
  if (angle < 0.f) {
    angle += TWO_PI;
  }
  return angle;
}

HallSensor *HallSensor::instance() { return s_instance_; }

// -----------------------------------------------------------------------------
// ISR path: called on TIM8 CH1 capture (hall transition edge)
void HallSensor::onTimerEdgeIsr(uint32_t capture_ticks) {
  // Read current hall combination and decode sector
  const uint8_t state = readState();
  last_state_ = state;
  if (state >= state_table_.size()) {
    return;
  }
  const int sector = state_table_[state];
  if (sector < 0) {
    return; // invalid combination
  }

  if (last_sector_ >= 0 && sector == last_sector_) {
    return; // duplicate edge without sector change
  }

  const uint32_t arr = (tim_ ? __HAL_TIM_GET_AUTORELOAD(tim_) : 0xFFFFu);

  // Compute dt in seconds using timer ticks with wrap-around handling.
  float dt_s = min_transition_dt_s_;
  if (last_capture_ticks_ == 0u) {
    dt_s = (tick_period_s_ > 0.f) ? tick_period_s_ : min_transition_dt_s_;
  } else {
    uint32_t dt_ticks = (capture_ticks >= last_capture_ticks_)
                            ? (capture_ticks - last_capture_ticks_)
                            : (arr - last_capture_ticks_ + capture_ticks + 1u);
    if (dt_ticks < min_transition_ticks_) {
      dt_ticks = min_transition_ticks_;
    }
    dt_s = static_cast<float>(dt_ticks) * tick_period_s_;
    if (dt_s <= kMinDt) {
      dt_s = (sample_period_ > kMinDt) ? sample_period_ : kMinDt;
    }
  }
  last_capture_ticks_ = capture_ticks;

  if (last_sector_ < 0) {
    // Seed with first valid sector
    last_sector_ = sector;
    mechanical_angle_wrapped_ = static_cast<float>(sector) * mechanical_step_;
    mechanical_angle_unwrapped_ = mechanical_angle_wrapped_;
    mech_velocity_ = 0.f;
    last_transition_time_s_ = nowSeconds();
    return;
  }

  if (sector != last_sector_) {
    int diff = sector - last_sector_;
    if (diff > 3) {
      diff -= 6;
    } else if (diff < -3) {
      diff += 6;
    }

    const float delta_mech = static_cast<float>(diff) * mechanical_step_;
    mechanical_angle_unwrapped_ += delta_mech;
    mechanical_angle_wrapped_ =
        wrapAngle(mechanical_angle_wrapped_ + delta_mech);

    float omega = delta_mech / dt_s;
    if (kMaxReasonableOmega > 0.f) {
      if (omega > kMaxReasonableOmega) {
        omega = kMaxReasonableOmega;
      } else if (omega < -kMaxReasonableOmega) {
        omega = -kMaxReasonableOmega;
      }
    }
    mech_velocity_ = omega;
    last_transition_time_s_ = nowSeconds();
    last_sector_ = sector;
  }
}

} // namespace kinematech

// HAL captures callback (C linkage) — dispatch to HallSensor instance
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM8 && kinematech::HallSensor::instance() != nullptr) {
    const uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    kinematech::HallSensor::instance()->onTimerEdgeIsr(cap);
  }
}
