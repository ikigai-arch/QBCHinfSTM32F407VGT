/* stm32f4xx_hal_msp.c -- inisialisasi level-rendah (keil_hal_v2).
 * TIM1, ADC1, GPIO daya/relai diatur langsung di register oleh App/hw_f407.c (sama dengan versi
 * Arduino). HAL di sini hanya untuk clock dasar dan USART3.
 */
#include "main.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef g = {0};
  if (huart->Instance == USART3) {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin       = UART_TX_PIN | UART_RX_PIN;   /* PB10 TX, PB11 RX */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(UART_GPIO_PORT, &g);

    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);    /* di bawah ISR kontrol ADC (prioritas 0) */
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
}
