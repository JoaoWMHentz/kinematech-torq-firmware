/*
 * utils.cpp
 *
 *  Created on: Nov 9, 2025
 *      Author: Firmware Team
 */

#include "utils/utils.hpp"

namespace kinematech::utils {

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

} // namespace kinematech::utils
