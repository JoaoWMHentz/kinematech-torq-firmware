/*
 * usb_communication.h
 * 
 * Camada de comunicação USB CDC para debug e telemetria
 * Created on: Oct 23, 2025
 */

#ifndef INC_USB_COMMUNICATION_H_
#define INC_USB_COMMUNICATION_H_

#include "main.h"
#include <stdint.h>

/* ========== TELEMETRY STRUCTURE ========== */
typedef struct {
    // Hall sensor
    uint8_t hall_state;
    float   hall_angle;
    float   hall_velocity;
    
    // Estado
    uint32_t uptime_ms;
    uint8_t  errors;
} Telemetry_t;

/* ========== PUBLIC FUNCTIONS ========== */

/**
 * @brief Inicializa o módulo de comunicação USB
 */
void USB_Comm_Init(void);

/**
 * @brief Envia telemetria via USB CDC
 * @param telem Ponteiro para estrutura de telemetria
 */
void USB_Comm_SendTelemetry(Telemetry_t* telem);

/**
 * @brief Envia string de debug via USB CDC
 * @param str String a ser enviada (deve terminar com \0)
 */
void USB_Comm_Print(const char* str);

/**
 * @brief Printf formatado via USB CDC
 * @note Usar com cuidado (pode ser lento)
 */
void USB_Comm_Printf(const char* fmt, ...);

/**
 * @brief Processa comandos recebidos via USB (chamado no loop principal)
 */
void USB_Comm_ProcessCommands(void);

#endif /* INC_USB_COMMUNICATION_H_ */
