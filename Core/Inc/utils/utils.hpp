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

} // namespace kinematech::utils

#endif /* INC_UTILS_UTILS_HPP_ */
