/* PIL lockstep via serial -- STM32F407VGT (Arduino IDE + STM32duino) -- v2
 *
 * Taruh file ini bersama controller_step.c, controller_step.h,
 * controller_coeffs.h di SATU folder sketch bernama "pil_stm32".
 * Tools -> USB support -> CDC (generic 'Serial' supersede U(S)ART)
 *
 * Paket Simulink/MATLAB -> STM32 (11 byte, little-endian):
 *   0xAA 0x55 CMD vref(float32) vo_meas(float32)
 *   CMD: 0 = jalankan 1 langkah, 1 = reset state kontroler
 * Paket STM32 -> MATLAB (10 byte):
 *   0xAA 0x55 duty(float32) cycles(uint32)
 *
 * v2: parser byte-per-byte tanpa peek() (peek di USB CDC tidak andal);
 *     LED berkedip tiap paket valid diterima (jika board punya LED_BUILTIN).
 */
#include <Arduino.h>

extern "C" {
#include "controller_step.h"
}

static uint8_t buf[11];
static uint8_t idx = 0;

static void process_packet(void) {
  uint8_t cmd = buf[2];
  float vref, vo;
  memcpy(&vref, &buf[3], 4);
  memcpy(&vo,   &buf[7], 4);

  float duty = 0.0f;
  uint32_t cycles = 0;

  if (cmd == 1) {
    controller_init();
  } else {
    uint32_t t0 = DWT->CYCCNT;
    duty = controller_step(vref, vo);
    cycles = DWT->CYCCNT - t0;
  }

  uint8_t tx[10];
  tx[0] = 0xAA; tx[1] = 0x55;
  memcpy(&tx[2], &duty,   4);
  memcpy(&tx[6], &cycles, 4);
  Serial.write(tx, 10);

#ifdef LED_BUILTIN
  digitalToggle(LED_BUILTIN);
#endif
}

void setup() {
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  Serial.begin(115200);          /* baud diabaikan oleh USB CDC */
  controller_init();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    if (idx == 0) {                       /* cari header 0xAA */
      if (b != 0xAA) continue;
    } else if (idx == 1) {                /* header kedua 0x55 */
      if (b != 0x55) {
        idx = (b == 0xAA) ? 1 : 0;        /* resync */
        continue;
      }
    }

    buf[idx++] = b;
    if (idx == 11) {
      idx = 0;
      process_packet();
    }
  }
}
