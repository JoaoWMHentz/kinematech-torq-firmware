/*
 * motor.h
 * 
 * Classe Motor - Contém configurações e estado do motor BLDC
 * Created on: Oct 24, 2025
 */

#ifndef INC_MOTOR_MOTOR_H_
#define INC_MOTOR_MOTOR_H_

#include "main.h"

/* ========== MOTOR CONFIG ========== */

// Configuração do motor (parâmetros elétricos fixos)
typedef struct {
    uint8_t pole_pairs;           // Número de pares de polos
    float kv_rating;              // Constante KV (RPM/V)
    float phase_resistance;       // Resistência de fase (Ohm)
    float phase_inductance;       // Indutância de fase (H)
    float voltage_limit;          // Tensão máxima (V)
    float current_limit;          // Corrente máxima (A)
    float velocity_limit_rpm;     // Velocidade máxima (RPM mecânico)
} MotorConfig_t;

/* ========== MOTOR CLASS ========== */

typedef struct {
    MotorConfig_t config;         // Configuração fixa do motor
} Motor_t;

/* ========== PUBLIC FUNCTIONS ========== */

// Inicialização do motor com configuração
void Motor_Init(Motor_t* motor, MotorConfig_t* config);

#endif /* INC_MOTOR_MOTOR_H_ */
