/*
 * math_utils.cpp
 *
 *  Created on: Oct 1, 2025
 *      Author: joaoh
 */

#include "utils/math_utils.hpp"

#include "definitions.h"

namespace kinematech::math {

float wrap_angle(float angle) {
    while (angle >= TWO_PI) {
        angle -= TWO_PI;
    }
    while (angle < 0.f) {
        angle += TWO_PI;
    }
    return angle;
}

} // namespace kinematech::math
