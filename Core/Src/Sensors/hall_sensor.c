/*
 * hall_sensor.c
 * 
 * Implementação do driver Hall Sensor usando TIM8 Hall Interface
 * Created on: Oct 23, 2025
 */

#include <Sensors/hall_sensor.h>
#include "config.h"
#include "tim.h"

// Timeout para considerar motor parado (ms)


// Mapeamento de estado físico → setor lógico (1-6)
// Sequência física: 1→5→4→6→2→3→1
// Sequência lógica: 1→2→3→4→5→6→1
static const uint8_t HALL_STATE_TO_SECTOR[8] = {
    0, 1, 5, 6, 3, 2, 4, 0  // [0]=inv, [1]=1, [2]=5, [3]=6, [4]=3, [5]=2, [6]=4, [7]=inv
};

// Tabela de ângulos elétricos (0 a 2π)
// Sequência: 1→5→4→6→2→3→1 (rotação horária)
// TODO: Implementar auto-calibração futura
static const float HALL_ANGLE_TABLE[8] = {
    0.0f,           // 0b000 - inválido
    0.0f,           // 0b001 (H:1) - 0°
    FOUR_PI_DIV_3,  // 0b010 (H:2) - 240°
    FIVE_PI_DIV_3,  // 0b011 (H:3) - 300°
    TWO_PI_DIV_3,   // 0b100 (H:4) - 120°
    PI_DIV_3,       // 0b101 (H:5) - 60°
    PI,             // 0b110 (H:6) - 180°
    0.0f            // 0b111 - inválido
};

// Detecta direção: +1=horário, -1=anti-horário, 0=inválido
static const int8_t HALL_TRANSITION_TABLE[8][8] = {
    {0,  0,  0,  0,  0,  0,  0,  0}, // De 0
    {0,  0,  0, -1,  0,  1,  0,  0}, // De 1: próx=5, ant=3
    {0,  0,  0,  1,  0,  0, -1,  0}, // De 2: próx=3, ant=6
    {0,  1, -1,  0,  0,  0,  0,  0}, // De 3: próx=1, ant=2
    {0,  0,  0,  0,  0, -1,  1,  0}, // De 4: próx=6, ant=5
    {0, -1,  0,  0,  1,  0,  0,  0}, // De 5: próx=4, ant=1
    {0,  0,  1,  0, -1,  0,  0,  0}, // De 6: próx=2, ant=4
    {0,  0,  0,  0,  0,  0,  0,  0}  // De 7
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
    hall->isr_counter = 0;
    hall->last_update_time = 0;
    
    hall->hall_state = Hall_ReadState();
    hall->last_hall_state = hall->hall_state;
    
    if (hall->hall_state >= 1 && hall->hall_state <= 6) {
        hall->angle_electrical = HALL_ANGLE_TABLE[hall->hall_state];
    }
    
    HAL_TIMEx_HallSensor_Start_IT(&htim8);
}

uint8_t Hall_ReadState(void) {
    // Ler estado Hall diretamente do registrador GPIOB->IDR
    // TIM8 Hall Interface usa os pinos em modo AF, mas IDR ainda reflete o estado físico
    uint32_t idr = GPIOB->IDR;
    uint8_t state = 0;
    
    // PB6 (HALL_A) → bit 0
    if (idr & GPIO_PIN_6) state |= 0x01;
    // PB8 (HALL_B) → bit 1
    if (idr & GPIO_PIN_8) state |= 0x02;
    // PB9 (HALL_C) → bit 2
    if (idr & GPIO_PIN_9) state |= 0x04;
    
    return state;
}

void Hall_ProcessData(HallSensor_t* hall) {
    // Verificar timeout: se passou muito tempo sem transição, motor parou
    uint32_t current_time = HAL_GetTick();
    if (current_time - hall->last_update_time > HALL_VELOCITY_TIMEOUT_MS) {
        hall->velocity_erpm = 0.0f;
        hall->direction = 0;
    }
    
    if (!hall->new_capture_flag) return;
    hall->new_capture_flag = 0;
    
    uint32_t period_us = hall->hall_capture;
    
    // Calcular velocidade (válido: 100μs a 1s = 10 a 10000 RPM)
    if (period_us > 100 && period_us < 1000000) {
        float time_seconds = period_us / 1000000.0f;
        float electrical_rps = (1.0f / 6.0f) / time_seconds; // 1/6 rotação por transição
        hall->velocity_erpm = electrical_rps * 60.0f;
        hall->last_update_time = current_time;  // Atualizar timestamp
    } else {
        hall->velocity_erpm = 0.0f;
    }
    
    hall->last_capture = hall->hall_capture;
    
    // Detectar direção e contar rotações
    uint8_t new_state = hall->hall_state;
    
    if (new_state != hall->last_hall_state && new_state >= 1 && new_state <= 6) {
        int8_t transition = HALL_TRANSITION_TABLE[hall->last_hall_state][new_state];
        
        if (transition != 0) {
            hall->direction = (transition > 0) ? 1 : 2;  // +1=horário, -1=anti-horário

            // Detecta quando completa uma rotação elétrica
            if (hall->last_hall_state == 6 && new_state == 1) {
                hall->electrical_rotations++;  // Horário
            } else if (hall->last_hall_state == 1 && new_state == 6) {
                hall->electrical_rotations--;  // Anti-horário
            }
        }

        hall->last_hall_state = new_state;
    }

    // Atualizar ângulo baseado na tabela
    if (hall->hall_state >= 1 && hall->hall_state <= 6) {
        hall->angle_electrical = HALL_ANGLE_TABLE[hall->hall_state];
    }
}

void Hall_TIM_CaptureCallback(HallSensor_t* hall) {
    hall->hall_capture = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);
    
    hall->hall_state = Hall_ReadState();
    hall->new_capture_flag = 1;
}

float Hall_GetAngle(HallSensor_t* hall) {
    return hall->angle_electrical;
}

float Hall_GetVelocity(HallSensor_t* hall) {
    return hall->velocity_erpm;
}

float Hall_GetMechanicalVelocity(HallSensor_t* hall) {
    // Converte eRPM para RPM mecânico
    // RPM_mecânico = eRPM / número_de_pares_de_polos
    return hall->velocity_erpm / (float)MOTOR_POLE_PAIRS;
}

uint8_t Hall_GetSector(HallSensor_t* hall) {
    return HALL_STATE_TO_SECTOR[hall->hall_state];
}

uint8_t Hall_GetDirection(HallSensor_t* hall) {
    return hall->direction;
}

uint8_t Hall_IsValid(HallSensor_t* hall) {
    return (hall->hall_state >= 1 && hall->hall_state <= 6);
}

/* ========== SENSOR INTERFACE IMPLEMENTATION ========== */

Sensor_t Hall_CreateSensorInterface(HallSensor_t* hall) {
    Sensor_t sensor;
    sensor.instance = hall;
    sensor.get_angle_electrical = (float (*)(void*))Hall_GetAngle;
    sensor.get_velocity_erpm = (float (*)(void*))Hall_GetVelocity;
    sensor.get_direction = (uint8_t (*)(void*))Hall_GetDirection;
    sensor.is_valid = (uint8_t (*)(void*))Hall_IsValid;
    return sensor;
}