/*
 * config.h
 * 
 * Configurações gerais do ESC
 * Created on: Oct 23, 2025
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"

/* ========== MOTOR PARAMETERS ========== */
#define MOTOR_POLE_PAIRS 7

/* ========== MATH CONSTANTS ========== */
#define PI               3.141593f    // π
#define TWO_PI           6.283185f    // 2π
#define PI_DIV_2         1.570796f    // π/2
#define PI_DIV_3         1.047198f    // π/3  (60°)
#define PI_DIV_6         0.523599f    // π/6  (30°)
#define TWO_PI_DIV_3     2.094395f    // 2π/3 (120°)
#define FOUR_PI_DIV_3    4.188790f    // 4π/3 (240°)
#define FIVE_PI_DIV_3    5.235988f    // 5π/3 (300°)
#define FIVE_PI_DIV_6    2.617994f    // 5π/6 (150°)

/* ========== CONTROL LOOP TIMING ========== */
#define FOC_LOOP_FREQUENCY_HZ   20000  // 20kHz - Loop de controle FOC
#define TELEMETRY_RATE_HZ       5    // 100Hz - Taxa de envio USB

/* ========== CURRENT SENSING (INA240) ========== */
#define CURRENT_SENSE_GAIN      20.0f  // Ganho do INA240 (V/V)
#define SHUNT_RESISTANCE        0.001f // 1mOhm
#define ADC_RESOLUTION          4096.0f // 12-bit ADC
#define ADC_VREF                3.3f    // Tensão de referência

/* ========== LIMITS ========== */
#define MAX_CURRENT_A           30.0f  // Corrente máxima (A)
#define MAX_DUTY_CYCLE          0.95f  // Duty cycle máximo (95%)

/* ========== DEBUG ========== */
#define DEBUG_ENABLED           1      // Habilita prints via USB CDC

#endif /* INC_CONFIG_H_ */
