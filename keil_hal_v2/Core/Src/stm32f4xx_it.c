/* stm32f4xx_it.c -- handler interrupt dasar (keil_hal_v2).
 * TIDAK ada di sini (didefinisikan di App/hw_f407.c, sama dengan versi Arduino):
 *   ADC_IRQHandler                      ISR kontrol 10 kHz (dipicu TIM1)
 *   HardFault/MemManage/BusFault/UsageFault_Handler   PWM mati + K1 dibuka, lalu tunggu watchdog
 */
#include "main.h"
#include "app_uart.h"

void NMI_Handler(void)        { while (1) { } }
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
