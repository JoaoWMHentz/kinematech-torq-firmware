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

static char usb_tx_buffer[256];
static uint8_t usb_rx_buffer[128];
static uint8_t rx_index = 0;

void USB_Comm_Init(void) {
    rx_index = 0;
    memset(usb_tx_buffer, 0, sizeof(usb_tx_buffer));
    memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));
}

void USB_Comm_SendTelemetry(Telemetry_t* telem) {
    int32_t vel_int = (int32_t)(telem->hall_velocity * 10.0f);
    
    snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
             "H:%d,Ang:%d,Vel:%ld,ISR:%lu,Time:%lu\r\n",
             telem->hall_sector,
             telem->hall_sector,
             vel_int,
             telem->isr_counter,
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
    // TODO: Parser de comandos (CAL, START, STOP, etc.)
}
