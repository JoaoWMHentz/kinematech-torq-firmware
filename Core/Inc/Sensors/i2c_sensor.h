/*
 * i2c_sensor.h
 * 
 * Exemplo de sensor I2C (encoder absoluto) - NÃO IMPLEMENTADO
 * Serve como template para futuros sensores
 * Created on: Oct 24, 2025
 */

#ifndef INC_SENSORS_I2C_SENSOR_H_
#define INC_SENSORS_I2C_SENSOR_H_

#include "main.h"
#include "Sensors/sensor.h"

/* ========== I2C SENSOR STRUCTURE ========== */

typedef struct {
    // Hardware configuration
    I2C_HandleTypeDef* hi2c;      // I2C handle
    uint8_t device_address;       // Endereço I2C do sensor
    
    // Sensor state
    float angle_electrical;       // Ângulo elétrico lido (rad, 0-2π)
    float velocity_erpm;          // Velocidade calculada (eRPM)
    uint8_t direction;            // Direção: 0=parado, 1=CW, 2=CCW
    uint8_t is_connected;         // Sensor conectado e respondendo
    
    // Timing
    uint32_t last_read_time;      // Timestamp da última leitura
} I2CSensor_t;

/* ========== PUBLIC FUNCTIONS ========== */

// Inicializa o sensor I2C
void I2C_Sensor_Init(I2CSensor_t* sensor);

// Atualiza leitura do sensor (chamar periodicamente)
void I2C_Sensor_Update(I2CSensor_t* sensor);

// Retorna ângulo elétrico (rad)
float I2C_Sensor_GetAngle(I2CSensor_t* sensor);

// Retorna velocidade (eRPM)
float I2C_Sensor_GetVelocity(I2CSensor_t* sensor);

// Retorna direção
uint8_t I2C_Sensor_GetDirection(I2CSensor_t* sensor);

// Verifica se sensor está válido
uint8_t I2C_Sensor_IsValid(I2CSensor_t* sensor);

/* ========== SENSOR INTERFACE ========== */

// Criar interface Sensor_t para este sensor I2C
Sensor_t I2C_CreateSensorInterface(I2CSensor_t* sensor);

#endif /* INC_SENSORS_I2C_SENSOR_H_ */
