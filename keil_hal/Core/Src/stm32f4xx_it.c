/* stm32f4xx_it.c -- handler interrupt dasar.
 * TIM1_UP_TIM10_IRQHandler ada di main.c (tick kontrol 10 kHz).
 */
#include "main.h"
#include "app_uart.h"

void NMI_Handler(void)        { while (1) { } }
void HardFault_Handler(void)  { TIM1->BDTR &= ~TIM_BDTR_MOE; while (1) { } }
void MemManage_Handler(void)  { TIM1->BDTR &= ~TIM_BDTR_MOE; while (1) { } }
void BusFault_Handler(void)   { TIM1->BDTR &= ~TIM_BDTR_MOE; while (1) { } }
void UsageFault_Handler(void) { TIM1->BDTR &= ~TIM_BDTR_MOE; while (1) { } }
void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void USART3_IRQHandler(void)
{
  uart_irq();
}
