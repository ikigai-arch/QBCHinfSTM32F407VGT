/* hw_f407.h -- lapisan perangkat keras STM32F407VGT untuk board QBC v2 (register-level).
 * Pin (dicek terhadap netlist KiCad v0.1, pad U6; lihat REVIEW_firmware_v1.md):
 *   PA8  TIM1_CH1  DRIVE_MOSFET -> R24 470 -> LED TLP250 (R26 10k pull-down) : aktif HIGH
 *   PA3  ADC1_IN3  V_OUT  (setelah relai K2!)
 *   PA5  ADC1_IN5  I_IN   (shunt R23 5 mOhm, jalur balik)
 *   PA4  ADC1_IN4  V_IN   (sebelum relai K1)
 *   PA2  ADC1_IN2  I_OUT  (shunt R29 20 mOhm)
 *   PA7  ADC1_IN7  TEMP1  (NTC)
 *   PC1  K1 Relay_Input  (aktif HIGH, D882, pull-down basis 5,1k -> OFF saat reset)
 *   PC0  K2 Relay_Output (idem)
 *   PE0  Kipas            (idem)
 *   PE6  TRIG (bebas di PCB)  pulsa 1 ms saat step Vref / perintah trig
 *   PE9  ISR  (bebas di PCB)  HIGH selama ISR kontrol (lihat saat cuplik & waktu eksekusi)
 */
#ifndef HW_F407_H
#define HW_F407_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

void     hw_safe_gpio_early(void);   /* PANGGIL PALING AWAL: semua keluaran daya LOW */
void     hw_init(void);              /* TIM1 PWM 10 kHz + ADC1 injected + NVIC; PWM tetap mati */
void     hw_iwdg_start(void);        /* watchdog ~100 ms (tidak bisa dimatikan setelah start) */
void     hw_service(uint32_t now_ms);/* dari loop: watchdog + deteksi ISR macet + statistik */
void     hw_kill(void);              /* PWM mati (MOE=0) + K1 dibuka, langsung di register */
uint32_t hw_isr_count(void);
uint32_t hw_overruns(void);
uint32_t hw_isr_cycles(void);
uint32_t hw_sysclk(void);
int      hw_clock_is_hse(void);
const char *hw_reset_cause(void);    /* dibaca sekali saat boot */
uint32_t qbc_hw_cycles(void);        /* DWT->CYCCNT */
void     SystemClock_Config(void);   /* HSE 8 MHz -> 168 MHz (cadangan HSI); Arduino memanggilnya sendiri */

#ifdef __cplusplus
}
#endif
#endif
