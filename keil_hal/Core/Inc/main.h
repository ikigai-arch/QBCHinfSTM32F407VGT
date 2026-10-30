/* main.h -- Firmware H-infinity QBC, versi STM32 HAL (Keil uVision / MDK-ARM)
 * Perilaku sama dengan hil_stm32/hil_stm32.ino (versi Arduino).
 */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

/* ===================== PIN (sama dengan PINMAP.md) ===================== */
/* PWM -> TLP250 / DRIVE_MOSFET : PA8 = TIM1_CH1 (AF1)                       */
#define PWM_GPIO_PORT      GPIOA
#define PWM_PIN            GPIO_PIN_8
/* V_OUT sensing         : PA3 = ADC1_IN3                                     */
#define VO_GPIO_PORT       GPIOA
#define VO_PIN             GPIO_PIN_3
#define VO_ADC_CHANNEL     ADC_CHANNEL_3
/* Fault dari proteksi    : PD8, input pull-down, aktif HIGH                  */
#define FAULT_GPIO_PORT    GPIOD
#define FAULT_PIN          GPIO_PIN_8
/* LED status            : PD9                                               */
#define LED_GPIO_PORT      GPIOD
#define LED_PIN            GPIO_PIN_9
/* Serial perintah/telemetri: USART3, PB10 = TX, PB11 = RX, 115200 8N1      */
#define UART_GPIO_PORT     GPIOB
#define UART_TX_PIN        GPIO_PIN_10
#define UART_RX_PIN        GPIO_PIN_11
#define UART_BAUD          115200U

void Error_Handler(void);

#endif /* MAIN_H */
