/* main.h -- Firmware board QBC v2, versi Keil uVision + STM32 HAL (keil_hal_v2).
 * Pin daya, sensing, relai, kipas, TRIG, ISR: lihat App/hw_f407.h (sama dengan versi Arduino).
 */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

/* Serial perintah/telemetri: USART3, PB10 = TX, PB11 = RX, 115200 8N1 (modul USB-TTL 3,3 V) */
#define UART_GPIO_PORT     GPIOB
#define UART_TX_PIN        GPIO_PIN_10
#define UART_RX_PIN        GPIO_PIN_11
#define UART_BAUD          115200U

void Error_Handler(void);

#endif /* MAIN_H */
