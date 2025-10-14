/*
 * utils.hpp
 *
 *  Created on: Nov 9, 2025
 *      Author: Firmware Team
 */

#ifndef INC_UTILS_UTILS_HPP_
#define INC_UTILS_UTILS_HPP_

#pragma once

#include <cstdint>

extern "C" {
#include "stm32g4xx_hal.h"
}

namespace kinematech::utils {

float timerInputClockHz(const TIM_HandleTypeDef* tim);

void cdc_print(const char* text);
void cdc_print(char value);
void cdc_print(int32_t value);
void cdc_print(float value);
void cdc_println();

} // namespace kinematech::utils

#endif /* INC_UTILS_UTILS_HPP_ */
