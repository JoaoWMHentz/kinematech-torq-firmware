/*
 * usb_communication.c
 * 
 * Implementação da camada de comunicação USB
 * Created on: Oct 23, 2025
 */

#include "usb_communication.h"
#include "usbd_cdc_if.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* ========== PRIVATE VARIABLES ========== */
static char usb_tx_buffer[256];
static uint8_t usb_rx_buffer[128];
static uint8_t rx_index = 0;

/* ========== PUBLIC FUNCTIONS ========== */

void USB_Comm_Init(void) {
    rx_index = 0;
    memset(usb_tx_buffer, 0, sizeof(usb_tx_buffer));
    memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));
}

void USB_Comm_SendTelemetry(Telemetry_t* telem) {
    // Formato CSV simplificado - usando inteiros para evitar problema com float/printf
    // Ang e Vel multiplicados por 100 para manter 2 casas decimais
    int32_t ang_int = (int32_t)(telem->hall_angle * 100.0f);
    int32_t vel_int = (int32_t)(telem->hall_velocity * 10.0f);
    
    snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
             "H:%d,Ang:%ld,Vel:%ld,ISR:%lu,Time:%lu\r\n",
             telem->hall_state,
             ang_int,      // Ângulo x100 (ex: 1.05 rad -> 105)
             vel_int,      // Velocidade x10 (ex: 123.4 eRPM -> 1234)
             telem->isr_counter,  // DEBUG: contador de ISR
             telem->uptime_ms);
    
    CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
}

void USB_Comm_Print(const char* str) {
    CDC_Transmit_FS((uint8_t*)str, strlen(str));
}

void USB_Comm_Printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(usb_tx_buffer, sizeof(usb_tx_buffer), fmt, args);
    va_end(args);
    
    CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
}

void USB_Comm_ProcessCommands(void) {
    // TODO: Implementar parser de comandos
    // Por enquanto, apenas placeholder
    // Comandos futuros: "CAL", "START", "STOP", etc.
}
