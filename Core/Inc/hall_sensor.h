/*
 * hall_sensor.h
 * 
 * Driver para sensores Hall 3-fases
 * Created on: Oct 23, 2025
 */

#ifndef INC_HALL_SENSOR_H_
#define INC_HALL_SENSOR_H_

#include "main.h"
#include <stdint.h>

/* ========== HALL SENSOR STRUCTURE ========== */
typedef struct {
    // Hardware state (atualizado na ISR do TIM8)
    volatile uint8_t hall_state;          // Estado atual (0-7)
    volatile uint32_t hall_capture;       // Valor capturado do TIM8 counter
    volatile uint32_t last_capture;       // Capture anterior
    volatile uint8_t new_capture_flag;    // Flag: nova transição detectada
    volatile uint32_t isr_counter;
    // Software state (processado no main loop)
    uint8_t last_hall_state;              // Estado anterior
    int32_t electrical_rotations;         // Contador de rotações elétricas
    float   angle_electrical;             // Ângulo elétrico estimado (0 a 2π)
    float   velocity_erpm;                // Velocidade elétrica (eRPM)
    uint8_t direction;                    // Direção: 0=parado, 1=horário, 2=anti-horário
} HallSensor_t;

/* ========== PUBLIC FUNCTIONS ========== */

/**
 * @brief Inicializa o sensor Hall
 */
void Hall_Init(HallSensor_t* hall);

/**
 * @brief Lê o estado atual dos sensores Hall
 * @return Estado Hall (0-7), 0 ou 7 = inválido
 */
uint8_t Hall_ReadState(void);

/**
 * @brief Processa dados do Hall (ângulo e velocidade)
 * @note Deve ser chamado no main loop (não em ISR!)
 */
void Hall_ProcessData(HallSensor_t* hall);

/**
 * @brief Callback ISR do TIM8 Hall Interface (uso interno)
 * @note Chamado automaticamente pela HAL, não chamar manualmente
 */
void Hall_TIM_CaptureCallback(HallSensor_t* hall);

/**
 * @brief Retorna o ângulo elétrico atual (radianos)
 * @return Ângulo entre 0 e 2*PI
 */
float Hall_GetAngle(HallSensor_t* hall);

/**
 * @brief Retorna a velocidade em eRPM
 * @return Velocidade elétrica (rotações elétricas por minuto)
 */
float Hall_GetVelocity(HallSensor_t* hall);

#endif /* INC_HALL_SENSOR_H_ */
