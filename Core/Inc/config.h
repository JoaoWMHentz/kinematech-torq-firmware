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
#define MOTOR_POLE_PAIRS        7      // Número de pares de polos

/* ========== MATH CONSTANTS ========== */
#define PI                      3.14159265358979323846f
#define TWO_PI                  6.28318530717958647692f
#define PI_DIV_2                1.57079632679489661923f  // π/2
#define PI_DIV_3                1.04719755119659774615f  // π/3
#define PI_DIV_6                0.52359877559829887308f  // π/6
#define TWO_PI_DIV_3            2.09439510239319549230f  // 2π/3
#define FOUR_PI_DIV_3           4.18879020478639098461f  // 4π/3
#define FIVE_PI_DIV_3           5.23598775598298873077f  // 5π/3
#define FIVE_PI_DIV_6           2.61799387799149436538f  // 5π/6

/* ========== CONTROL LOOP TIMING ========== */
#define FOC_LOOP_FREQUENCY_HZ   20000  // 20kHz - Loop de controle FOC
#define TELEMETRY_RATE_HZ       5    // 100Hz - Taxa de envio USB

/* ========== CURRENT SENSING (INA240) ========== */
#define CURRENT_SENSE_GAIN      20.0f  // Ganho do INA240 (V/V)
#define SHUNT_RESISTANCE        0.001f // 1mOhm
#define ADC_RESOLUTION          4096.0f // 12-bit ADC
#define ADC_VREF                3.3f    // Tensão de referência

// Conversão ADC -> Corrente (A)
// I = (ADC_value * VREF / ADC_RES - VREF/2) / (GAIN * R_shunt)
#define ADC_TO_CURRENT(adc_val) (((float)(adc_val) * ADC_VREF / ADC_RESOLUTION - ADC_VREF/2.0f) / (CURRENT_SENSE_GAIN * SHUNT_RESISTANCE))

/* ========== LIMITS ========== */
#define MAX_CURRENT_A           30.0f  // Corrente máxima (A)
#define MAX_DUTY_CYCLE          0.95f  // Duty cycle máximo (95%)

/* ========== DEBUG ========== */
#define DEBUG_ENABLED           1      // Habilita prints via USB CDC

#endif /* INC_CONFIG_H_ */
