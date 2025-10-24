/*
 * diagnostics.c
 * 
 * Diagnóstico do TIM8 Hall Sensor para debug
 */

#include "diagnostics.h"
#include "usb_communication.h"
#include "tim.h"
#include <stdio.h>

void TIM8_PrintDiagnostics(void) {
    char buffer[512];
    
    USB_Comm_Print("\r\n===== TIM8 DIAGNOSTICS =====\r\n");
    
    // Control Register 1
    snprintf(buffer, sizeof(buffer), "TIM8->CR1:    0x%08lX\r\n", TIM8->CR1);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  CEN (bit 0): %ld (Counter Enable)\r\n", (TIM8->CR1 & TIM_CR1_CEN) ? 1 : 0);
    USB_Comm_Print(buffer);
    
    // Slave Mode Control Register
    snprintf(buffer, sizeof(buffer), "TIM8->SMCR:   0x%08lX\r\n", TIM8->SMCR);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  SMS (bits 0-2): %ld (Expected: 4 = Reset Mode)\r\n", (TIM8->SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  TS (bits 4-6):  %ld (Expected: 5 = TI1F_ED)\r\n", (TIM8->SMCR & TIM_SMCR_TS_Msk) >> TIM_SMCR_TS_Pos);
    USB_Comm_Print(buffer);
    
    // DMA/Interrupt Enable Register
    snprintf(buffer, sizeof(buffer), "TIM8->DIER:   0x%08lX\r\n", TIM8->DIER);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  CC1IE (bit 1):  %ld (Capture/Compare 1 Interrupt Enable)\r\n", (TIM8->DIER & TIM_DIER_CC1IE) ? 1 : 0);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  CC2IE (bit 2):  %ld (Capture/Compare 2 Interrupt Enable)\r\n", (TIM8->DIER & TIM_DIER_CC2IE) ? 1 : 0);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  TIE (bit 6):    %ld (Trigger Interrupt Enable)\r\n", (TIM8->DIER & TIM_DIER_TIE) ? 1 : 0);
    USB_Comm_Print(buffer);
    
    // Status Register
    snprintf(buffer, sizeof(buffer), "TIM8->SR:     0x%08lX\r\n", TIM8->SR);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  CC1IF (bit 1):  %ld (Capture/Compare 1 Interrupt Flag)\r\n", (TIM8->SR & TIM_SR_CC1IF) ? 1 : 0);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  CC2IF (bit 2):  %ld (Capture/Compare 2 Interrupt Flag)\r\n", (TIM8->SR & TIM_SR_CC2IF) ? 1 : 0);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  TIF (bit 6):    %ld (Trigger Interrupt Flag)\r\n", (TIM8->SR & TIM_SR_TIF) ? 1 : 0);
    USB_Comm_Print(buffer);
    
    // Capture/Compare Mode Register 1
    snprintf(buffer, sizeof(buffer), "TIM8->CCMR1:  0x%08lX\r\n", TIM8->CCMR1);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  CC1S (bits 0-1): %ld (Expected: 1 = TI1 mapped on TI1FP1)\r\n", (TIM8->CCMR1 & TIM_CCMR1_CC1S_Msk) >> TIM_CCMR1_CC1S_Pos);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  IC1F (bits 4-7): %ld (Input Capture 1 Filter)\r\n", (TIM8->CCMR1 & TIM_CCMR1_IC1F_Msk) >> TIM_CCMR1_IC1F_Pos);
    USB_Comm_Print(buffer);
    
    // Capture/Compare Enable Register
    snprintf(buffer, sizeof(buffer), "TIM8->CCER:   0x%08lX\r\n", TIM8->CCER);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  CC1E (bit 0):   %ld (Capture/Compare 1 Enable)\r\n", (TIM8->CCER & TIM_CCER_CC1E) ? 1 : 0);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  CC1P (bit 1):   %ld (Capture/Compare 1 Polarity)\r\n", (TIM8->CCER & TIM_CCER_CC1P) ? 1 : 0);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  CC2E (bit 4):   %ld (Capture/Compare 2 Enable)\r\n", (TIM8->CCER & TIM_CCER_CC2E) ? 1 : 0);
    USB_Comm_Print(buffer);
    
    // Prescaler and Counter
    snprintf(buffer, sizeof(buffer), "TIM8->PSC:    %lu (Prescaler)\r\n", TIM8->PSC);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "TIM8->ARR:    %lu (Auto-Reload)\r\n", TIM8->ARR);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "TIM8->CNT:    %lu (Counter Value)\r\n", TIM8->CNT);
    USB_Comm_Print(buffer);
    
    // NVIC Status
    uint32_t nvic_iser = NVIC->ISER[TIM8_CC_IRQn / 32];
    uint32_t nvic_bit = 1UL << (TIM8_CC_IRQn % 32);
    snprintf(buffer, sizeof(buffer), "\r\nNVIC TIM8_CC_IRQn (%d):\r\n", TIM8_CC_IRQn);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  Enabled: %s\r\n", (nvic_iser & nvic_bit) ? "YES" : "NO");
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  Priority: %lu\r\n", NVIC_GetPriority(TIM8_CC_IRQn));
    USB_Comm_Print(buffer);
    
    // GPIO State
    snprintf(buffer, sizeof(buffer), "\r\nGPIOB->IDR: 0x%04lX\r\n", GPIOB->IDR);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  HALL_A (PB6): %ld\r\n", (GPIOB->IDR & GPIO_PIN_6) ? 1 : 0);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  HALL_B (PB8): %ld\r\n", (GPIOB->IDR & GPIO_PIN_8) ? 1 : 0);
    USB_Comm_Print(buffer);
    snprintf(buffer, sizeof(buffer), "  HALL_C (PB9): %ld\r\n", (GPIOB->IDR & GPIO_PIN_9) ? 1 : 0);
    USB_Comm_Print(buffer);
    
    USB_Comm_Print("===========================\r\n\r\n");
}
