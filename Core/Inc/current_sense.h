/*
 * current_sense.h
 * 
 * Driver para leitura de corrente (INA240 + ADC)
 * Created on: Oct 23, 2025
 */

#ifndef INC_CURRENT_SENSE_H_
#define INC_CURRENT_SENSE_H_

#include "main.h"
#include <stdint.h>

/* ========== CURRENT SENSE STRUCTURE ========== */
typedef struct {
    float i_a;              // Corrente fase A (Amperes)
    float i_b;              // Corrente fase B (Amperes)
    float i_c;              // Corrente fase C (Amperes)
    
    uint16_t adc_raw_a;     // Valor bruto ADC fase A
    uint16_t adc_raw_b;     // Valor bruto ADC fase B
    uint16_t adc_raw_c;     // Valor bruto ADC fase C
    
    float offset_a;         // Offset de calibração fase A
    float offset_b;         // Offset de calibração fase B
    float offset_c;         // Offset de calibração fase C
    
    uint8_t calibrated;     // Flag de calibração
} CurrentSense_t;

/* ========== PUBLIC FUNCTIONS ========== */

/**
 * @brief Inicializa o módulo de leitura de corrente
 * @note Deve ser chamado após MX_ADC_Init()
 */
void CurrentSense_Init(CurrentSense_t* cs);

/**
 * @brief Calibra os offsets de corrente (motor parado, sem corrente)
 * @param samples Número de amostras para média (recomendado: 1000)
 */
void CurrentSense_Calibrate(CurrentSense_t* cs, uint16_t samples);

/**
 * @brief Lê as correntes das 3 fases
 * @note Deve ser chamado no loop de controle (20kHz)
 */
void CurrentSense_Read(CurrentSense_t* cs);

/**
 * @brief Retorna corrente da fase A em Amperes
 */
float CurrentSense_GetPhaseA(CurrentSense_t* cs);

/**
 * @brief Retorna corrente da fase B em Amperes
 */
float CurrentSense_GetPhaseB(CurrentSense_t* cs);

/**
 * @brief Retorna corrente da fase C em Amperes (calculada: Ic = -(Ia + Ib))
 */
float CurrentSense_GetPhaseC(CurrentSense_t* cs);

#endif /* INC_CURRENT_SENSE_H_ */
