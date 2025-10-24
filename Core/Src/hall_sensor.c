/*
 * hall_sensor.c
 * 
 * Implementação do driver Hall Sensor usando TIM8 Hall Interface
 * Created on: Oct 23, 2025
 */

#include "hall_sensor.h"
#include "config.h"
#include "tim.h"
#include <math.h>
#include <definitions.h>

/* Tabela de ângulos elétricos para cada estado Hall (6-step)
 * Estado Hall: HallC | HallB | HallA
 * Ângulos em radianos (0 a 2π)
 */
static const float HALL_ANGLE_TABLE[8] = {
    0.0f,                    // 0b000 - Estado inválido
    0.0f,                    // 0b001 - 0°
    M_PI / 3.0f,             // 0b010 - 60°
    M_PI / 6.0f,             // 0b011 - 30°
    2.0f * M_PI / 3.0f,      // 0b100 - 120°
    5.0f * M_PI / 6.0f,      // 0b101 - 150°
    4.0f * M_PI / 3.0f,      // 0b110 - 240°
    0.0f                     // 0b111 - Estado inválido
};

/* Tabela de transições válidas (detecta direção)
 * +1 = horário, -1 = anti-horário, 0 = inválido
 */
static const int8_t HALL_TRANSITION_TABLE[8][8] = {
//   0   1   2   3   4   5   6   7
    {0,  0,  0,  0,  0,  0,  0,  0}, // De 0 (inválido)
    {0,  0,  1,  0, -1,  0,  0,  0}, // De 1
    {0, -1,  0,  1,  0,  0,  0,  0}, // De 2
    {0,  0, -1,  0,  1,  0,  0,  0}, // De 3
    {0,  1,  0, -1,  0,  1,  0,  0}, // De 4
    {0,  0,  0,  0, -1,  0,  1,  0}, // De 5
    {0,  0,  0,  0,  0, -1,  0,  1}, // De 6
    {0,  0,  0,  0,  0,  0,  0,  0}  // De 7 (inválido)
};

/* ========== PUBLIC FUNCTIONS ========== */

void Hall_Init(HallSensor_t* hall) {
    hall->hall_state = 0;
    hall->last_hall_state = 0;
    hall->electrical_rotations = 0;
    hall->angle_electrical = 0.0f;
    hall->velocity_erpm = 0.0f;
    hall->direction = 0;
    hall->hall_capture = 0;
    hall->last_capture = 0;
    hall->new_capture_flag = 0;
    hall->isr_counter = 0;  // DEBUG: inicializar contador
    
    // Leitura inicial do estado Hall
    hall->hall_state = Hall_ReadState();
    hall->last_hall_state = hall->hall_state;
    
    // Setar ângulo inicial baseado no estado Hall atual
    if (hall->hall_state >= 1 && hall->hall_state <= 6) {
        hall->angle_electrical = HALL_ANGLE_TABLE[hall->hall_state];
    }
    
    // Iniciar TIM8 em modo Hall Sensor com interrupção
    HAL_TIMEx_HallSensor_Start_IT(&htim8);
    
    // ===== FIX CRÍTICO: Habilitar TODAS as interrupções possíveis para debug =====
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC1);    // Capture/Compare 1 (já habilitado pela HAL)
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC2);    // Capture/Compare 2 (usado pelo commutation)
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_TRIGGER); // Trigger (edge detection)
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_COM);    // Commutation event
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE); // Counter overflow (pode ajudar no debug)
}

uint8_t Hall_ReadState(void) {
    uint8_t state = 0;
    
    // Ler os 3 pinos Hall (PB6, PB8, PB9)
    // IMPORTANTE: Como estão em modo AF (TIM8), ler direto do registrador IDR
    if (GPIOB->IDR & HALL_A_Pin) state |= 0x01; // Bit 0
    if (GPIOB->IDR & HALL_B_Pin) state |= 0x02; // Bit 1
    if (GPIOB->IDR & HALL_C_Pin) state |= 0x04; // Bit 2
    
    return state;
}

void Hall_ProcessData(HallSensor_t* hall) {
    // Processar apenas se houver nova captura
    if (!hall->new_capture_flag) {
        return;
    }
    
    hall->new_capture_flag = 0; // Limpar flag
    
    // Calcular velocidade baseado no delta de captures
    uint32_t delta_capture = hall->hall_capture - hall->last_capture;
    
    if (delta_capture > 0 && delta_capture < 1000000) { // Timeout de 1 segundo (1MHz timer)
        // Timer roda a 1MHz (1μs por tick)
        // Cada transição Hall = 60° elétrico = 1/6 rotação elétrica
        float time_seconds = delta_capture / 1000000.0f;
        float electrical_rps = (1.0f / 6.0f) / time_seconds;
        hall->velocity_erpm = electrical_rps * 60.0f; // Converter para eRPM
    } else {
        hall->velocity_erpm = 0.0f; // Timeout ou primeira leitura
    }
    
    // Verificar direção baseado na transição
    uint8_t new_state = hall->hall_state;
    
    if (new_state != hall->last_hall_state && new_state >= 1 && new_state <= 6) {
        int8_t transition = HALL_TRANSITION_TABLE[hall->last_hall_state][new_state];
        
        if (transition != 0) {
            // Transição válida
            hall->direction = (transition > 0) ? 1 : 2;
            
            // Atualizar contador de rotações
            if (hall->last_hall_state == 6 && new_state == 1) {
                hall->electrical_rotations++; // Completou rotação horária
            } else if (hall->last_hall_state == 1 && new_state == 6) {
                hall->electrical_rotations--; // Completou rotação anti-horária
            }
        }
        
        hall->last_hall_state = new_state;
    }
    
    // Atualizar ângulo baseado na tabela
    if (hall->hall_state >= 1 && hall->hall_state <= 6) {
        hall->angle_electrical = HALL_ANGLE_TABLE[hall->hall_state];
    }
    
    hall->last_capture = hall->hall_capture;
}

void Hall_TIM_CaptureCallback(HallSensor_t* hall) {
    // ===== ISR ULTRA RÁPIDA (executada a cada transição Hall) =====
    // Objetivo: capturar dados e sair imediatamente (~1-2μs)
    
    // DEBUG: LED já piscado na callback do main.c
    // Não duplicar aqui para evitar overhead
    
    // 1. Ler estado Hall atual
    hall->hall_state = Hall_ReadState();
    
    // 2. Capturar valor do contador do TIM8
    hall->hall_capture = __HAL_TIM_GET_COUNTER(&htim8);
    
    // 3. Setar flag para processar no main loop
    hall->new_capture_flag = 1;
    
    // FIM da ISR! Processamento pesado vai no main loop
}

float Hall_GetAngle(HallSensor_t* hall) {
    return hall->angle_electrical;
}

float Hall_GetVelocity(HallSensor_t* hall) {
    return hall->velocity_erpm;
}
