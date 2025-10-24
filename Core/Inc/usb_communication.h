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
    uint8_t hall_state;      // Estado físico do Hall (1-7)
    uint8_t hall_sector;     // Setor lógico (1-6)
    float   hall_angle;
    float   hall_velocity;
    uint32_t isr_counter;
    uint32_t uptime_ms;
    uint8_t  errors;
} Telemetry_t;

// Inicializa o módulo de comunicação USB
void USB_Comm_Init(void);

// Envia telemetria via USB CDC
void USB_Comm_SendTelemetry(Telemetry_t* telem);

// Envia string de debug via USB CDC
void USB_Comm_Print(const char* str);

// Printf formatado via USB CDC (usar com cuidado - pode ser lento)
void USB_Comm_Printf(const char* fmt, ...);

// Processa comandos recebidos via USB (chamar no loop principal)
void USB_Comm_ProcessCommands(void);

#endif /* INC_USB_COMMUNICATION_H_ */
