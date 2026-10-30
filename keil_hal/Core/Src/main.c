/* main.c -- Firmware kontroler H-infinity untuk board QBC (versi STM32 HAL)
 * Target : STM32F407VGT (DevEBox), Keil uVision MDK-ARM (ARM Compiler 6)
 *
 * Port dari hil_stm32/hil_stm32.ino (Arduino/STM32duino). Pin, parameter,
 * proteksi, dan perintah serial SAMA. Bedanya hanya:
 *   - serial lewat USART3 (PB10 TX / PB11 RX, 115200 8N1) + modul USB-TTL,
 *     bukan USB CDC di port micro-USB DevEBox.
 *   - clock diatur sendiri: 168 MHz dari HSE 8 MHz (cadangan: HSI 16 MHz).
 * controller_step.c / .h / controller_coeffs.h dipakai TANPA diubah.
 *
 * Alur: TIM1 membangkitkan PWM 10 kHz di PA8. Tiap update event TIM1
 * (akhir periode PWM) loop utama menjalankan satu langkah kontrol:
 *   baca Vo (ADC PA3) -> controller_step(vref, vo) -> duty -> CCR1 TIM1.
 * CCR1 ber-preload, jadi duty baru berlaku di periode berikutnya
 * (setara blok 1/z di model Simulink).
 *
 * Perintah serial (diakhiri Enter / newline), PWM selalu MATI saat boot:
 *   e        enable  (mulai kontrol, ramp vref dari Vo terukur)
 *   d        disable / reset kontroler / hapus FAULT
 *   v<angka> set vref target dalam volt, contoh: v50   (0..VREF_MAX)
 *   k<angka> faktor kalibrasi Vo (pengali), contoh: k0.98
 *   m<angka> paksa Vo = angka volt (0..400), mengabaikan ADC. contoh: m30
 *   a        kembali membaca Vo dari ADC asli
 *   p        cetak status
 * Telemetri tiap TELEMETRY_MS ms:
 *   T,ms,state,vref,vo,duty,cycles,overrun,manual
 *   state: 0=OFF 1=RUN 2=FAULT ; manual: 1 = Vo disuntik lewat serial
 */
#include "main.h"
#include "app_uart.h"
#include <stdlib.h>
#include <stdbool.h>

#include "controller_step.h"
#include "controller_coeffs.h"     /* CTRL_TS */

/* ===================== PARAMETER (sama dengan versi Arduino) ============ */
#define FSW_HZ          10000u       /* HARUS = 1/CTRL_TS (Ts = 100 us)       */
#define DUTY_HW_MAX     0.65f        /* batas atas duty di hardware           */
#define VREF_MAX        150.0f       /* batas vref uji                        */
#define OVP_V           165.0f       /* proteksi lebih-tegangan (latch)       */
#define OVP_COUNT       3u           /* sampel berturut-turut utk trip        */
#define FAULT_ACTIVE    GPIO_PIN_SET /* level aktif PD8 -- CEK DESAIN         */
#define RAMP_V_PER_S    6000.0f      /* = 150 V / 25 ms, sama dgn model       */
#define TELEMETRY_MS    50u
#define VO_MANUAL_MAX   400.0f

/* Skala Vo: volt di terminal output = (volt di pin ADC) * VO_SCALE.
 * ISI DARI RANGKAIAN DIFF AMP + LPF YANG SEBENARNYA, lalu kalibrasi dengan
 * multimeter memakai perintah k<angka>. Nilai di bawah hanya placeholder. */
#define VO_SCALE        88.96f
#define ADC_VREF        3.3f
#define ADC_MAX         4095.0f

/* ===================== HANDLE & STATE ===================== */
enum { ST_OFF = 0, ST_RUN = 1, ST_FAULT = 2 };

TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;

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
static bool     voManual   = false;
static float    voManualV  = 0.0f;
static bool     clockFromHse = false;

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void TIM1_PWM_Init(void);
static void ADC1_Init(void);
static void DWT_Init(void);

/* ===================== ISR TIM1 UPDATE ===================== */
/* Nama handler sesuai tabel vektor startup_stm32f407xx.s */
void TIM1_UP_TIM10_IRQHandler(void)
{
  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR = ~TIM_SR_UIF;              /* hapus flag update */
    if (tickFlag) overruns++;
    tickFlag = true;
  }
}

/* ===================== HELPER ===================== */
static void setDuty(float d)
{
  if (!(d > 0.0f)) d = 0.0f;              /* juga menangkap NaN */
  if (d > DUTY_HW_MAX) d = DUTY_HW_MAX;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(d * (float)(arr + 1u) + 0.5f));
  lastDuty = d;
}

static uint32_t adcReadOnce(void)
{
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK) return 0;
  return HAL_ADC_GetValue(&hadc1);
}

static float readVo(void)
{
  if (voManual) return voManualV;
  uint32_t s = adcReadOnce();
  s += adcReadOnce();
  return ((float)s * 0.5f) / ADC_MAX * ADC_VREF * VO_SCALE * voCal;
}

static void trip(void)
{
  setDuty(0.0f);
  state = ST_FAULT;
}

static void controlTick(void)
{
  float vo = readVo();
  lastVo = vo;

  if (state != ST_RUN) {
    setDuty(0.0f);
    return;
  }

  if (vo > OVP_V) ovpCnt++; else ovpCnt = 0;
  if (HAL_GPIO_ReadPin(FAULT_GPIO_PORT, FAULT_PIN) == FAULT_ACTIVE || ovpCnt >= OVP_COUNT) {
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

static void printStatus(void)
{
  uart_puts("state=");        uart_put_u32(state);
  uart_puts(" vref_target="); uart_put_float(vrefTarget, 2);
  uart_puts(" vo=");          uart_put_float(lastVo, 2);
  uart_puts(" duty=");        uart_put_float(lastDuty, 4);
  uart_puts(" cal=");         uart_put_float(voCal, 4);
  uart_puts(" manual=");      uart_put_u32(voManual ? 1u : 0u);
  uart_puts(" sysclk=");      uart_put_u32(HAL_RCC_GetSysClockFreq());
  uart_puts(clockFromHse ? " (HSE)" : " (HSI)");
  uart_puts(" arr=");         uart_put_u32(__HAL_TIM_GET_AUTORELOAD(&htim1));
  uart_puts("\r\n");
}

static void handleCommand(char *line)
{
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
      float v = (float)atof(line + 1);
      if (v < 0.0f) v = 0.0f;
      if (v > VREF_MAX) v = VREF_MAX;
      vrefTarget = v;
      break;
    }
    case 'k': {
      float k = (float)atof(line + 1);
      if (k > 0.1f && k < 10.0f) voCal = k;
      break;
    }
    case 'm': {
      float v = (float)atof(line + 1);
      if (v < 0.0f) v = 0.0f;
      if (v > VO_MANUAL_MAX) v = VO_MANUAL_MAX;
      voManualV = v;
      voManual = true;
      break;
    }
    case 'a':
      voManual = false;
      break;
    case 'p':
      printStatus();
      break;
    default:
      break;
  }
}

static void pollSerial(void)
{
  static char buf[24];
  static uint8_t n = 0;
  int c;
  while ((c = uart_getc()) >= 0) {
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; handleCommand(buf); n = 0; }
    } else if (n < sizeof(buf) - 1u) {
      buf[n++] = (char)c;
    }
  }
}

static void sendTelemetry(uint32_t now)
{
  uart_puts("T,");          uart_put_u32(now);
  uart_putc(',');           uart_put_u32(state);
  uart_putc(',');           uart_put_float(vrefNow, 2);
  uart_putc(',');           uart_put_float(lastVo, 2);
  uart_putc(',');           uart_put_float(lastDuty, 4);
  uart_putc(',');           uart_put_u32(lastCycles);
  uart_putc(',');           uart_put_u32(overruns);
  uart_putc(',');           uart_put_u32(voManual ? 1u : 0u);
  uart_puts("\r\n");
}

/* ===================== MAIN ===================== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  GPIO_Init();
  uart_init();
  ADC1_Init();
  DWT_Init();
  controller_init();
  TIM1_PWM_Init();                       /* PWM jalan dengan duty 0 = MATI */

  uart_puts("\r\nQBC H-inf HAL firmware siap. PWM OFF. Perintah: e d v k m a p\r\n");

  uint32_t lastTx = HAL_GetTick();

  while (1) {
    pollSerial();

    if (tickFlag) {
      tickFlag = false;
      controlTick();
    }

    uint32_t now = HAL_GetTick();
    GPIO_PinState led = GPIO_PIN_RESET;
    if (state == ST_RUN) led = GPIO_PIN_SET;
    else if (state == ST_FAULT) led = ((now / 100u) & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, led);

    if (now - lastTx >= TELEMETRY_MS) {
      lastTx = now;
      sendTelemetry(now);
    }
  }
}

/* ===================== INISIALISASI ===================== */

/* 168 MHz: HSE 8 MHz / M8 * N336 / P2. Jika HSE gagal start, pakai HSI 16 MHz
 * (M16) supaya board tetap jalan dengan frekuensi PWM yang sama. */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState       = RCC_HSE_ON;
  osc.PLL.PLLState   = RCC_PLL_ON;
  osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM       = 8;
  osc.PLL.PLLN       = 336;
  osc.PLL.PLLP       = RCC_PLLP_DIV2;
  osc.PLL.PLLQ       = 7;
  if (HAL_RCC_OscConfig(&osc) == HAL_OK) {
    clockFromHse = true;
  } else {
    RCC_OscInitTypeDef hsi = {0};
    hsi.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    hsi.HSIState            = RCC_HSI_ON;
    hsi.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    hsi.PLL.PLLState        = RCC_PLL_ON;
    hsi.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    hsi.PLL.PLLM            = 16;
    hsi.PLL.PLLN            = 336;
    hsi.PLL.PLLP            = RCC_PLLP_DIV2;
    hsi.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&hsi) != HAL_OK) Error_Handler();
    clockFromHse = false;
  }

  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;      /* HCLK  168 MHz */
  clk.APB1CLKDivider = RCC_HCLK_DIV4;        /* PCLK1  42 MHz */
  clk.APB2CLKDivider = RCC_HCLK_DIV2;        /* PCLK2  84 MHz -> TIM1 168 MHz */
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void GPIO_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* LED status PD9 */
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
  g.Pin   = LED_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &g);

  /* Fault PD8: input pull-down */
  g.Pin   = FAULT_PIN;
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(FAULT_GPIO_PORT, &g);
}

static uint32_t tim1ClockHz(void)
{
  /* Timer di APB2 mendapat 2x PCLK2 bila prescaler APB2 != 1 */
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  return ((RCC->CFGR & RCC_CFGR_PPRE2) == 0u) ? pclk2 : 2u * pclk2;
}

static void TIM1_PWM_Init(void)
{
  TIM_OC_InitTypeDef oc = {0};
  TIM_BreakDeadTimeConfigTypeDef bdt = {0};
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 0;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = tim1ClockHz() / FSW_HZ - 1u;   /* 16799 @168 MHz */
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

  oc.OCMode       = TIM_OCMODE_PWM1;      /* HIGH selama counter < CCR1 */
  oc.Pulse        = 0;                    /* duty 0 = output LOW */
  oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode   = TIM_OCFAST_DISABLE;
  oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  /* (ConfigChannel juga menyalakan preload CCR1 -> duty berlaku periode berikutnya) */

  bdt.OffStateRunMode  = TIM_OSSR_DISABLE;
  bdt.OffStateIDLEMode = TIM_OSSI_DISABLE;
  bdt.LockLevel        = TIM_LOCKLEVEL_OFF;
  bdt.DeadTime         = 0;
  bdt.BreakState       = TIM_BREAK_DISABLE;
  bdt.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  bdt.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt) != HAL_OK) Error_Handler();

  /* PA8 baru dijadikan output timer setelah CCR1 = 0 */
  g.Pin       = PWM_PIN;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;              /* pull-down ada di PCB */
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(PWM_GPIO_PORT, &g);

  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();  /* juga set MOE */
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

static void ADC1_Init(void)
{
  ADC_ChannelConfTypeDef ch = {0};

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 21 MHz */
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  ch.Channel      = VO_ADC_CHANNEL;
  ch.Rank         = 1;
  ch.SamplingTime = ADC_SAMPLETIME_84CYCLES;   /* 4 us; aman untuk sumber ber-impedansi sedang */
  if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();
}

static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Error_Handler(void)
{
  /* Gagal inisialisasi: matikan output PWM dan kedipkan LED cepat. */
  __disable_irq();
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  while (1) {
    GPIOD->ODR ^= LED_PIN;
    for (volatile uint32_t i = 0; i < 1000000u; i++) { }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
  Error_Handler();
}
#endif
