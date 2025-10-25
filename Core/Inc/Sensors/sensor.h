/*
 * sensor.h
 * 
 * Interface genérica para sensores de posição/velocidade
 * Todos os sensores (Hall, Encoder, I2C, etc) devem implementar esta interface
 * Created on: Oct 24, 2025
 */

#ifndef INC_SENSORS_SENSOR_H_
#define INC_SENSORS_SENSOR_H_

#include "main.h"

/* ========== SENSOR INTERFACE ========== */

// Interface genérica que qualquer sensor deve implementar
typedef struct {
    void* instance;  // Ponteiro para instância concreta (HallSensor_t, I2CSensor_t, etc)
    
    // Obter ângulo elétrico (rad, 0-2π)
    float (*get_angle_electrical)(void* instance);
    
    // Obter velocidade elétrica (eRPM)
    float (*get_velocity_erpm)(void* instance);
    
    // Obter direção (0=parado, 1=CW, 2=CCW)
    uint8_t (*get_direction)(void* instance);
    
    // Verificar se sensor está válido
    uint8_t (*is_valid)(void* instance);
    
} Sensor_t;

#endif /* INC_SENSORS_SENSOR_H_ */
