/* app_uart.c -- USART3 (PB10 TX / PB11 RX) dengan ring buffer RX & TX.
 *
 * HAL dipakai untuk inisialisasi (HAL_UART_Init + MSP). Pengiriman/penerimaan
 * byte memakai interrupt RXNE/TXE langsung di register, supaya telemetri
 * tidak memblokir loop kontrol 10 kHz (HAL_UART_Transmit bersifat blocking).
 */
#include "main.h"
#include "app_uart.h"

#define TX_SIZE 512u          /* harus pangkat 2 */
#define RX_SIZE 64u           /* harus pangkat 2 */

UART_HandleTypeDef huart3;

static volatile uint8_t  txBuf[TX_SIZE];
static volatile uint16_t txHead = 0, txTail = 0;
static volatile uint8_t  rxBuf[RX_SIZE];
static volatile uint16_t rxHead = 0, rxTail = 0;

void uart_init(void)
{
  huart3.Instance          = USART3;
  huart3.Init.BaudRate     = UART_BAUD;
  huart3.Init.WordLength   = UART_WORDLENGTH_8B;
  huart3.Init.StopBits     = UART_STOPBITS_1;
  huart3.Init.Parity       = UART_PARITY_NONE;
  huart3.Init.Mode         = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* aktifkan interrupt terima; interrupt kirim diaktifkan saat ada data */
  SET_BIT(USART3->CR1, USART_CR1_RXNEIE);
}

void uart_irq(void)
{
  uint32_t sr = USART3->SR;

  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
    uint8_t b = (uint8_t)USART3->DR;          /* baca SR lalu DR = hapus flag */
    if (sr & USART_SR_RXNE) {
      uint16_t next = (uint16_t)((rxHead + 1u) & (RX_SIZE - 1u));
      if (next != rxTail) {
        rxBuf[rxHead] = b;
        rxHead = next;
      }
    }
  }

  if ((USART3->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
    if (txTail != txHead) {
      USART3->DR = txBuf[txTail];
      txTail = (uint16_t)((txTail + 1u) & (TX_SIZE - 1u));
    } else {
      CLEAR_BIT(USART3->CR1, USART_CR1_TXEIE);
    }
  }
}

int uart_getc(void)
{
  if (rxTail == rxHead) return -1;
  uint8_t b = rxBuf[rxTail];
  rxTail = (uint16_t)((rxTail + 1u) & (RX_SIZE - 1u));
  return b;
}

void uart_putc(char c)
{
  uint16_t next = (uint16_t)((txHead + 1u) & (TX_SIZE - 1u));
  if (next == txTail) return;                 /* penuh: buang, jangan blok */
  txBuf[txHead] = (uint8_t)c;
  txHead = next;
  SET_BIT(USART3->CR1, USART_CR1_TXEIE);
}

void uart_puts(const char *s)
{
  while (*s) uart_putc(*s++);
}

void uart_put_u32(uint32_t v)
{
  char tmp[11];
  int n = 0;
  do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v && n < 10);
  while (n) uart_putc(tmp[--n]);
}

/* cetak float tanpa printf("%f") -- aman untuk semua pustaka C Keil */
void uart_put_float(float v, int decimals)
{
  uint32_t scale = 1u, ip, fp;
  int i;

  if (v != v) { uart_puts("nan"); return; }
  if (v < 0.0f) { uart_putc('-'); v = -v; }
  if (v > 4.0e9f) { uart_puts("ovf"); return; }

  for (i = 0; i < decimals; i++) scale *= 10u;
  {
    float r = v * (float)scale + 0.5f;
    if (r > 4.0e9f) { uart_puts("ovf"); return; }
    uint32_t t = (uint32_t)r;
    ip = t / scale;
    fp = t % scale;
  }
  uart_put_u32(ip);
  if (decimals > 0) {
    char tmp[10];
    uart_putc('.');
    for (i = decimals - 1; i >= 0; i--) { tmp[i] = (char)('0' + (fp % 10u)); fp /= 10u; }
    for (i = 0; i < decimals; i++) uart_putc(tmp[i]);
  }
}
