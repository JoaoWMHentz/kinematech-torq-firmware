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

// ===== System / PWM settings ==============================================================
#define SYSCLK_HZ   170000000.0f   // System clock [Hz]
#define PWM_FREQ_HZ 20000.0f       // PWM frequency [Hz] (center-aligned)
#define DEADTIME_NS 10             // Target dead-time (~200–300 ns)

// ARR calculation (center-aligned: f_pwm = f_tim / (2 * (ARR + 1)))
#define TIM1_ARR ((uint32_t)((SYSCLK_HZ / (2.0f * PWM_FREQ_HZ)) - 1.0f))

// ===== DC bus / SVPWM settings ===========================================================
#define VBUS_V        30.0f  // DC bus voltage [V] (adjust to match hardware)
#define SVPWM_LIMIT_K 0.866f // Voltage limit factor (≈ √3/2)

// ===== Open-loop parameters ==============================================================
#define OL_UQ_V         10.0f  // Initial q-axis voltage [V] (torque command)
#define OL_FREQ_ELEC_HZ 5.0f // Electrical frequency [Hz]
#define POLE_PAIRS      15    // Motor pole pairs

// ===== Closed-loop (FOC) PI gains ========================================================
#define FOC_VEL_KP 0.4f
#define FOC_VEL_KI 12.0f
#define FOC_POS_KP 0.0f
#define FOC_POS_KI 0.0f
#define FOC_CURR_KP 0.0f
#define FOC_CURR_KI 0.0f

// ===== Pin mapping (TIM1) ===============================================================
// CH1  -> PA8  (PA_HIN)   / CH1N -> PB13 (PA_LIN)
// CH2  -> PA9  (PB_HIN)   / CH2N -> PB14 (PB_LIN)
// CH3  -> PA10 (PC_HIN)   / CH3N -> PB15 (PC_LIN)

// ===== Math settings ====================================================================
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TWO_PI    (2.0f * (float)M_PI)
#define _SQRT3_2  0.86602540378f
#define HALL_PRINT_PERIOD 100U // 10 Hz update rate
#define KRAD_TO_MIRAD 1000.0f
#define KRAD_TO_RPM 60.0f / TWO_PI

#endif /* INC_DEFINITIONS_H_ */
