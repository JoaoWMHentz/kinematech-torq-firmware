/*
 * utils.cpp
 *
 *  Created on: Nov 9, 2025
 *      Author: Firmware Team
 */

#include "utils/utils.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>

extern "C" {
#include "usbd_cdc_if.h"
}

namespace kinematech::utils {

namespace {

constexpr uint16_t kMaxCdcPrintLen = 120U;
std::array<uint8_t, kMaxCdcPrintLen> tx_buffer {};

void transmitBuffer(const uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length == 0U) {
        return;
    }

    const uint16_t capped_length = std::min<uint16_t>(length, kMaxCdcPrintLen);
    std::memcpy(tx_buffer.data(), buffer, capped_length);

    (void)CDC_Transmit_FS(tx_buffer.data(), capped_length);
}

} // namespace

float timerInputClockHz(const TIM_HandleTypeDef* tim) {
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

void cdc_print(const char* text) {
    if (text == nullptr) {
        return;
    }

    const size_t len = std::strlen(text);
    if (len == 0U) {
        return;
    }

    const size_t capped_len = std::min(len, static_cast<size_t>(kMaxCdcPrintLen));
    transmitBuffer(reinterpret_cast<const uint8_t*>(text), static_cast<uint16_t>(capped_len));
}

void cdc_print(char value) {
    uint8_t buffer[1] = { static_cast<uint8_t>(value) };
    transmitBuffer(buffer, 1U);
}

void cdc_print(int32_t value) {
    std::array<char, 16> buffer {};
    const auto written = std::snprintf(buffer.data(), buffer.size(), "%ld", static_cast<long>(value));
    if (written <= 0) {
        return;
    }

    const auto length = static_cast<size_t>(written);
    transmitBuffer(reinterpret_cast<const uint8_t*>(buffer.data()), static_cast<uint16_t>(length));
}

void cdc_print(float value) {
    std::array<char, 32> buffer {};
    const auto written = std::snprintf(buffer.data(), buffer.size(), "%f", static_cast<double>(value));
    if (written <= 0) {
        return;
    }

    const auto length = static_cast<size_t>(written);
    transmitBuffer(reinterpret_cast<const uint8_t*>(buffer.data()), static_cast<uint16_t>(length));
}

void cdc_println() {
    char newline[] = "\r\n";
    transmitBuffer(reinterpret_cast<const uint8_t*>(newline), 2U);
}

} // namespace kinematech::utils
