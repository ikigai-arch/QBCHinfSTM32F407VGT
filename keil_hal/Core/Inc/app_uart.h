/* app_uart.h -- USART3 ber-buffer (interrupt), tidak pernah memblokir loop kontrol. */
#ifndef APP_UART_H
#define APP_UART_H

#include <stdint.h>

void uart_init(void);                 /* panggil setelah HAL_Init + clock     */
int  uart_getc(void);                 /* -1 jika tidak ada data               */
void uart_putc(char c);               /* byte dibuang jika buffer TX penuh    */
void uart_puts(const char *s);
void uart_put_u32(uint32_t v);
void uart_put_float(float v, int decimals);
void uart_irq(void);                  /* dipanggil dari USART3_IRQHandler     */

#endif /* APP_UART_H */
