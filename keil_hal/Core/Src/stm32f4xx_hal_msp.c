/* stm32f4xx_hal_msp.c -- inisialisasi level-rendah (clock & pin) per periferal.
 * TIM1/PA8 diatur langsung di main.c (TIM1_PWM_Init) supaya urutannya jelas:
 * CCR1 = 0 dulu, baru PA8 dijadikan output timer.
 */
#include "main.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef g = {0};
  if (hadc->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin  = VO_PIN;                 /* PA3 = ADC1_IN3 */
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VO_GPIO_PORT, &g);
  }
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

    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);    /* di bawah TIM1 (prioritas 0) */
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
}
