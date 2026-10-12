/* hil_stm32.ino -- Firmware kontroler H-infinity untuk board QBC
 * Target : STM32F407VGT (DevEBox) -- Arduino IDE + STM32duino
 * Setelan: Tools -> USB support -> CDC (generic 'Serial' supersede U(S)ART)
 *
 * Satu folder sketch "hil_stm32" berisi: hil_stm32.ino, controller_step.c,
 * controller_step.h, controller_coeffs.h  (tiga file terakhir TIDAK diubah,
 * sama persis dengan yang dipakai di MIL-2 / SIL / PIL).
 *
 * Alur: TIM1 membangkitkan PWM 10 kHz di PA8. Tiap akhir periode PWM
 * (update event) loop() menjalankan satu langkah kontrol:
 *   baca Vo (ADC PA3) -> controller_step(vref, vo) -> duty -> CCR TIM1.
 * CCR ber-preload, jadi duty baru berlaku di periode berikutnya
 * (setara blok 1/z di model Simulink).
 *
 * Perintah serial (115200, newline), PWM selalu MATI saat boot:
 *   e        enable  (mulai kontrol, ramp vref dari Vo terukur)
 *   d        disable / reset kontroler / hapus FAULT
 *   v<angka> set vref target dalam volt, contoh: v50   (0..VREF_MAX)
 *   k<angka> faktor kalibrasi Vo (pengali), contoh: k0.98
 *   p        cetak status
 * Telemetri tiap TELEMETRY_MS ms:
 *   T,ms,state,vref,vo,duty,cycles,overrun
 *   state: 0=OFF 1=RUN 2=FAULT
 */
#include <Arduino.h>

extern "C" {
#include "controller_step.h"
#include "controller_coeffs.h"     /* CTRL_TS */
}

/* ===================== PIN (bekukan bersama Mas Tino) ===================== */
#define PIN_PWM     PA8     /* TIM1_CH1 -> input TLP250 (pull-down di PCB)   */
#define PIN_ADC_VO  PA3     /* ADC1_IN3 : Vout setelah diff amp + LPF analog */
#define PIN_FAULT   PD8     /* input fault dari proteksi PCB                 */
#define PIN_STATUS  PD9     /* LED status (opsional)                         */

/* ===================== PARAMETER ===================== */
#define FSW_HZ          10000        /* HARUS = 1/CTRL_TS (Ts = 100 us)       */
#define DUTY_HW_MAX     0.65f        /* batas atas duty di hardware           */
#define VREF_MAX        150.0f       /* batas vref uji                        */
#define OVP_V           165.0f       /* proteksi lebih-tegangan (latch)       */
#define OVP_COUNT       3            /* sampel berturut-turut utk trip        */
#define FAULT_LEVEL     HIGH         /* level aktif PIN_FAULT -- CEK DESAIN   */
#define RAMP_V_PER_S    6000.0f      /* = 150 V / 25 ms, sama dgn model;
                                        untuk hardware boleh diperlambat      */
#define TELEMETRY_MS    50

/* Skala Vo: volt di terminal output = (volt di pin ADC) * VO_SCALE.
 * ISI DARI RANGKAIAN DIFF AMP + LPF YANG SEBENARNYA, lalu kalibrasi dengan
 * multimeter memakai perintah k<angka>.  Nilai di bawah hanya placeholder
 * (pembagi 4x220k : 10k  => 1/0.01124). */
#define VO_SCALE        88.96f
#define ADC_VREF        3.3f         /* VDDA DevEBox                          */
#define ADC_MAX         4095.0f

/* ===================== STATE ===================== */
enum { ST_OFF = 0, ST_RUN = 1, ST_FAULT = 2 };

static HardwareTimer *pwm;
static volatile bool     tickFlag = false;
static volatile uint32_t overruns = 0;

static uint8_t  state      = ST_OFF;
static float    vrefTarget = 0.0f;
static float    vrefNow    = 0.0f;
static float    voCal      = 1.0f;
static float    lastVo     = 0.0f;
static float    lastDuty   = 0.0f;
static uint32_t lastCycles = 0;
static uint8_t  ovpCnt     = 0;

/* dipanggil di akhir tiap periode PWM (update event TIM1) */
static void onPeriod(void) {
  if (tickFlag) overruns++;
  tickFlag = true;
}

static void setDuty(float d) {
  if (!(d > 0.0f)) d = 0.0f;              /* juga menangkap NaN */
  if (d > DUTY_HW_MAX) d = DUTY_HW_MAX;
  uint32_t ovf = pwm->getOverflow(TICK_FORMAT);
  pwm->setCaptureCompare(1, (uint32_t)(d * (float)ovf + 0.5f), TICK_COMPARE_FORMAT);
  lastDuty = d;
}

static float readVo(void) {
  uint32_t s = analogRead(PIN_ADC_VO);
  s += analogRead(PIN_ADC_VO);
  return ((float)s * 0.5f) / ADC_MAX * ADC_VREF * VO_SCALE * voCal;
}

static void trip(void) {
  setDuty(0.0f);
  state = ST_FAULT;
}

static void controlTick(void) {
  float vo = readVo();
  lastVo = vo;

  if (state != ST_RUN) {
    setDuty(0.0f);
    return;
  }

  if (vo > OVP_V) ovpCnt++; else ovpCnt = 0;
  if (digitalRead(PIN_FAULT) == FAULT_LEVEL || ovpCnt >= OVP_COUNT) {
    trip();
    return;
  }

  /* ramp vref (soft-start), laju sama dengan Rate Limiter di model */
  float step = RAMP_V_PER_S * CTRL_TS;
  if (vrefNow < vrefTarget)      { vrefNow += step; if (vrefNow > vrefTarget) vrefNow = vrefTarget; }
  else if (vrefNow > vrefTarget) { vrefNow -= step; if (vrefNow < vrefTarget) vrefNow = vrefTarget; }

  uint32_t t0 = DWT->CYCCNT;
  float duty = controller_step(vrefNow, vo);
  lastCycles = DWT->CYCCNT - t0;

  setDuty(duty);
}

static void printStatus(void) {
  Serial.print(F("state=")); Serial.print(state);
  Serial.print(F(" vref_target=")); Serial.print(vrefTarget, 2);
  Serial.print(F(" vo=")); Serial.print(lastVo, 2);
  Serial.print(F(" duty=")); Serial.print(lastDuty, 4);
  Serial.print(F(" cal=")); Serial.println(voCal, 4);
}

static void handleCommand(char *line) {
  switch (line[0]) {
    case 'e':
      if (state == ST_OFF) {
        controller_init();
        vrefNow = readVo();              /* mulai ramp dari Vo terukur */
        ovpCnt = 0;
        state = ST_RUN;
      }
      break;
    case 'd':
      setDuty(0.0f);
      controller_init();
      vrefNow = 0.0f;
      state = ST_OFF;
      break;
    case 'v': {
      float v = atof(line + 1);
      if (v < 0.0f) v = 0.0f;
      if (v > VREF_MAX) v = VREF_MAX;
      vrefTarget = v;
      break;
    }
    case 'k': {
      float k = atof(line + 1);
      if (k > 0.1f && k < 10.0f) voCal = k;
      break;
    }
    case 'p':
      printStatus();
      break;
    default:
      break;
  }
}

static void pollSerial(void) {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; handleCommand(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void setup() {
  pinMode(PIN_FAULT, INPUT_PULLDOWN);
  pinMode(PIN_STATUS, OUTPUT);
  analogReadResolution(12);

  Serial.begin(115200);                  /* baud diabaikan oleh USB CDC */
  controller_init();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  pwm = new HardwareTimer(TIM1);
  pwm->setPWM(1, PIN_PWM, FSW_HZ, 0, onPeriod);   /* duty 0 = PWM mati */
}

void loop() {
  pollSerial();

  if (tickFlag) {
    tickFlag = false;
    controlTick();
  }

  digitalWrite(PIN_STATUS, state == ST_RUN ? HIGH : (state == ST_FAULT ? (millis() / 100) & 1 : LOW));

  static uint32_t lastTx = 0;
  uint32_t now = millis();
  if (now - lastTx >= TELEMETRY_MS) {
    lastTx = now;
    if (Serial) {
      Serial.print(F("T,"));  Serial.print(now);
      Serial.print(',');      Serial.print(state);
      Serial.print(',');      Serial.print(vrefNow, 2);
      Serial.print(',');      Serial.print(lastVo, 2);
      Serial.print(',');      Serial.print(lastDuty, 4);
      Serial.print(',');      Serial.print(lastCycles);
      Serial.print(',');      Serial.println(overruns);
    }
  }
}
