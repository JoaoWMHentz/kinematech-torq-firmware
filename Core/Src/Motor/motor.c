/*
 * motor.c
 * 
 * Implementação da classe Motor
 * Created on: Oct 24, 2025
 */

#include "Motor/motor.h"
#include <string.h>

void Motor_Init(Motor_t* motor, MotorConfig_t* config) {
    // Copiar configuração
    memcpy(&motor->config, config, sizeof(MotorConfig_t));
}
