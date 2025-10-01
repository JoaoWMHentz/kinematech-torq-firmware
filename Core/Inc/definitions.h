/*
 * definitions.h
 *  Created on: Sep 20, 2025
 *      Author: joaoh
 *
 *  Centralized system constants used by the ESC firmware.
 *  These values describe the MCU clock, PWM carrier, motor defaults,
 *  and other tuning parameters consumed by the SVPWM driver.
 */

#ifndef INC_DEFINITIONS_H_
#define INC_DEFINITIONS_H_

// ===== MCU / timing configuration ========================================================
#define SYSCLK_HZ   170000000.0f   // System clock driving the timers [Hz]
#define PWM_FREQ_HZ 20000.0f       // Desired PWM carrier frequency [Hz]
#define DEADTIME_NS 10             // Half-bridge deadtime budget [ns]
#define ARM_MATH_CM4               // Always enable Cortex-M4 tuned CMSIS-DSP

// Center-aligned timer: f_pwm = f_tim / (2 * (ARR + 1))
#define TIM1_ARR ((uint32_t)((SYSCLK_HZ / (2.0f * PWM_FREQ_HZ)) - 1.0f))

// ===== Motor / power defaults ============================================================
#define VBUS_V            20.0f  // Nominal DC bus used for duty normalization [V]
#define SVPWM_LIMIT_K     0.866f // Linear modulation limit factor (≈ √3 / 2)
#define POLE_PAIRS        15     // Motor electrical pole pairs
#define OL_UQ_V           10.0f  // Default open-loop q-axis voltage command [V]
#define OL_FREQ_ELEC_HZ   10.0f  // Default electrical frequency in open loop [Hz]
#define VELOCITY_TARGET_RAD_S 15.0f // Initial closed-loop velocity target [rad/s]

// ===== Control gains =====================================================================
#define FOC_VEL_KP  0.4f   // Velocity-loop proportional gain
#define FOC_VEL_KI 18.0f   // Velocity-loop integral gain
#define FOC_POS_KP  0.0f   // Position-loop proportional gain
#define FOC_POS_KI  0.0f   // Position-loop integral gain
#define FOC_CURR_KP 0.0f   // Current-loop proportional gain
#define FOC_CURR_KI 0.0f   // Current-loop integral gain

// ===== Pin mapping (TIM1) ===============================================================
// CH1  -> PA8  (PA_HIN)   / CH1N -> PB13 (PA_LIN)
// CH2  -> PA9  (PB_HIN)   / CH2N -> PB14 (PB_LIN)
// CH3  -> PA10 (PC_HIN)   / CH3N -> PB15 (PC_LIN)

// ===== Math helpers =====================================================================
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TWO_PI           (2.0f * (float)M_PI)    // Full rotation in radians
#define TWO_OVER_THREE   0.6666666667f           // Clarke alpha-beta scaling coefficient
#define ONE_OVER_SQRT3   0.5773502692f           // 1 / sqrt(3) for Clarke transform
#define _SQRT3_2         0.86602540378f          // sqrt(3) / 2 for SVPWM projections
#define KRAD_TO_MIRAD    1000.0f                 // Milli-radians per radian
#define KRAD_TO_RPM      (60.0f / TWO_PI)        // Convert rad/s to RPM

// ===== Hall sensor defaults =============================================================
#define HALL_PRINT_PERIOD        100U    // Diagnostic print cadence (10 Hz)
#define HALL_ELECTRICAL_STEP_RAD (TWO_PI / 6.0f) // Hall edge separation [rad]
#define HALL_MIN_DT_S            1e-7f   // Minimum delta-time guard [s]
#define HALL_DEFAULT_STALE_S     0.05f   // Timeout before velocity zeroing [s]
#define HALL_MAX_REASONABLE_OMEGA 2000.0f // Velocity guardrail [rad/s]
#define HALL_GUARD_MULTIPLIER    1.6f    // Stale timeout multiplier vs last edge
#define HALL_MAX_ADVANCE_FACTOR  0.9f    // Limit for predictive advance vs step

#endif /* INC_DEFINITIONS_H_ */
