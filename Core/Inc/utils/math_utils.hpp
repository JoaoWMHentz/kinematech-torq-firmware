/*
 * math_utils.hpp
 *
 *  Created on: Oct 1, 2025
 *      Author: joaoh
 */

#ifndef INC_UTILS_MATH_UTILS_HPP_
#define INC_UTILS_MATH_UTILS_HPP_

#pragma once

namespace kinematech::math {

template <typename T>
constexpr T clamp(T value, T min_val, T max_val) {
    if (max_val < min_val) {
        const T tmp = min_val;
        min_val = max_val;
        max_val = tmp;
    }

    if (value < min_val) {
        return min_val;
    }
    if (value > max_val) {
        return max_val;
    }
    return value;
}

float wrap_angle(float angle);

} // namespace kinematech::math

#endif /* INC_UTILS_MATH_UTILS_HPP_ */
