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

/* ========== CONTROL LOOP TIMING ========== */
#define FOC_LOOP_FREQUENCY_HZ   20000  // 20kHz - Loop de controle FOC
#define TELEMETRY_RATE_HZ       100    // 100Hz - Taxa de envio USB

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
