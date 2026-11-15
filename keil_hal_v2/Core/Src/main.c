/* main.c -- Firmware board QBC v2 (H-inf / PI-FR / MPSO-PI), versi Keil uVision + STM32 HAL.
 *
 * Logika SAMA PERSIS dengan hil_stm32_v2 (Arduino): semua file di App/ adalah salinan identik
 * dari folder hil_stm32_v2 (qbc_core, qbc_config, ctrl_mux, qbc_sim, hw_f407, pengendali).
 * Yang berbeda hanya file ini dan app_uart.c:
 *   - serial lewat USART3 (PB10 TX, PB11 RX, 115200 8N1) + modul USB-TTL 3,3 V,
 *     bukan USB CDC di port micro-USB DevEBox;
 *   - USART tidak punya sinyal DTR, jadi deteksi "link putus" memakai heartbeat (perintah "hb").
 *
 * Kontrol, proteksi, relai, dan ramp berjalan di ADC_IRQHandler (hw_f407.c) yang dipicu TIM1
 * 10 kHz. Loop di bawah hanya mengurus teks serial, watchdog, dan heartbeat.
 * Saat boot: PWM mati (PA8 LOW), K1/K2 terbuka, kipas mati, status OFF.
 */
#include "main.h"
#include "app_uart.h"
#include "qbc_core.h"
#include "qbc_config.h"
#include "hw_f407.h"
#include <string.h>

#define KEIL_FW_NOTE "keil_hal_v2 (USART3 115200)"

/* ---------- heartbeat pengganti DTR ----------
 * hb_ms = 0 (default): link dianggap selalu tersambung (seperti "link keep" di versi Arduino;
 *                      terminal biasa seperti PuTTY tidak mengirim apa-apa saat diam).
 * hb_ms > 0          : link dianggap putus bila tidak ada satu byte pun diterima selama hb_ms.
 *                      qbc_core lalu melakukan DISARM 1 s kemudian (LINK_LOSS_MS).
 *                      tools/qbc_term.py mengirim "\r" tiap 200 ms dan menyalakan "hb 1000". */
static uint32_t hb_ms = 0;
static uint32_t last_rx_ms = 0;

static char rxl[80];
static uint8_t rxn = 0;
static char rep[1400];

static void puts_n(const char *s, int n) { int i; for (i = 0; i < n; i++) uart_putc(s[i]); }

static int handle_local(const char *l)
{
  /* "hb <ms>" ditangani di sini (khusus versi USART) */
  if ((l[0] == 'h' || l[0] == 'H') && (l[1] == 'b' || l[1] == 'B') && (l[2] == ' ' || l[2] == 0)) {
    uint32_t v = 0;
    const char *p = l + 2;
    while (*p == ' ') p++;
    if (*p == 0) {
      uart_puts("HB ");
      uart_put_u32(hb_ms);
      uart_puts(hb_ms ? " ms\r\n" : " (mati: link dianggap selalu tersambung)\r\n");
      return 1;
    }
    while (*p >= '0' && *p <= '9') { v = v * 10u + (uint32_t)(*p - '0'); p++; }
    if (*p != 0 || (v != 0 && (v < 200u || v > 10000u))) { uart_puts("ERR hb 0 | 200..10000 ms\r\n"); return 1; }
    hb_ms = v;
    last_rx_ms = HAL_GetTick();
    uart_puts("OK\r\n");
    return 1;
  }
  return 0;
}

static void handle_line(void)
{
  rxl[rxn] = 0;
  if (!handle_local(rxl) && qbc_command(rxl, rep, sizeof(rep))) uart_puts(rep);
  rxn = 0;
}

static void poll_rx(uint32_t now)
{
  int c;
  while ((c = uart_getc()) >= 0) {
    last_rx_ms = now;
    if (c == '!' && rxn == 0) {                 /* darurat: langsung, tanpa menunggu Enter */
      rxl[0] = '!'; rxn = 1; handle_line();
      continue;
    }
    if (c == '\n' || c == '\r') { if (rxn > 0) handle_line(); }
    else if (rxn < sizeof(rxl) - 1u) rxl[rxn++] = (char)c;
  }
}

int main(void)
{
  static char line[256];
  int n, k;

  hw_safe_gpio_early();            /* PA8, K1, K2, kipas, TRIG, ISR = LOW sebelum apa pun */
  HAL_Init();                      /* SysTick 1 ms (HAL_GetTick) */
  SystemClock_Config();            /* hw_f407.c: HSE 8 MHz -> 168 MHz, cadangan HSI */
  uart_init();
  qbc_init();
  hw_init();                       /* TIM1 jalan dengan MOE = 0 (PWM LOW), ADC injected + ISR aktif */
  hw_iwdg_start();

  uart_puts("\r\n# " QBC_FW_VERSION " / " KEIL_FW_NOTE "\r\n# reset: ");
  uart_puts(hw_reset_cause());
  uart_puts(hw_clock_is_hse() == 1 ? "  clock: HSE 168 MHz\r\n" : "  clock: HSI (HSE GAGAL) - cek kristal\r\n");
  uart_puts("# PWM OFF, K1/K2 terbuka. Ketik help. Heartbeat: hb (lihat README).\r\n");
  uart_puts("# T,ms,state,ctrl,flags,vref,vo,vin,iin,iout,temp,duty,vo_min,vo_max,faults,isr_us,isr_us_max,ctrl_cyc,ovr,sat_ms\r\n");
  last_rx_ms = HAL_GetTick();

  while (1) {
    uint32_t now = HAL_GetTick();

    hw_service(now);               /* watchdog (hanya bila ISR hidup) + deteksi ISR macet */
    poll_rx(now);
    qbc_set_link(hb_ms == 0u || (uint32_t)(now - last_rx_ms) < hb_ms);

    /* event (prioritas), lalu keluaran raw/dump, lalu telemetri; buang bila buffer hampir penuh */
    while (uart_tx_free() > 300u && (n = qbc_poll_event(line, sizeof(line))) > 0) puts_n(line, n);
    for (k = 0; k < 8 && uart_tx_free() > 300u; k++) {
      if ((n = qbc_poll_output(line, sizeof(line))) <= 0) break;
      puts_n(line, n);
    }
    if (uart_tx_free() > 300u && (n = qbc_poll_telemetry(now, line, sizeof(line))) > 0) puts_n(line, n);
    if (uart_tx_dropped() && uart_tx_free() > 100u) {
      uart_puts("# ");
      uart_put_u32(uart_tx_dropped());
      uart_puts(" byte teks dibuang (serial lambat)\r\n");
      uart_tx_dropped_clear();
    }
  }
}

void Error_Handler(void)
{
  /* Gagal inisialisasi HAL/UART: PWM mati, K1/K2 dibuka, lalu diam (watchdog akan me-reset bila aktif). */
  __disable_irq();
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  GPIOC->BSRR = (1u << (1u + 16u)) | (1u << (0u + 16u));
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
  Error_Handler();
}
#endif
