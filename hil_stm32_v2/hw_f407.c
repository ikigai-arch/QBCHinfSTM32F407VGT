/* hw_f407.c -- lihat hw_f407.h. Register-level supaya urutan dan timing jelas.
 *
 * Rantai waktu per periode PWM (Ts = 100 us, TIM1 up-counting, ARR = 16799 @168 MHz):
 *   CNT = 0      : update event -> CCR1 (duty) & CCR4 (saat cuplik) dari preload berlaku
 *   CNT = CCR4   : OC4REF (PWM mode 2) naik -> TRGO -> ADC1 injected mulai
 *                  JSQ1 V_OUT, JSQ2 I_IN, JSQ3 V_IN, JSQ4 I_OUT (28 siklus @21 MHz -> 1,9 us/kanal)
 *   + ~7,6 us    : JEOC -> ADC_IRQHandler -> qbc_isr() -> CCR1/CCR4 baru (preload)
 *   CNT = ARR    : update event berikutnya -> duty baru berlaku  (= unit delay z^-1 di MIL-2/PIL)
 * Jika ISR selesai setelah update event, duty baru tertunda satu periode lagi -> dihitung "overrun".
 */
#include "hw_f407.h"
#include "qbc_core.h"
#include "qbc_config.h"
#if defined(ARDUINO)
#include "stm32_def.h"
#elif defined(USE_HAL_DRIVER)
#include "stm32f4xx_hal.h"          /* keil_hal_v2 (Keil uVision / arm-none-eabi-gcc + HAL) */
#else
#include "stm32f4xx.h"
#endif
#if defined(ARDUINO) || defined(USE_HAL_DRIVER)
#define QBC_HAL_CLOCK 1             /* SystemClock_Config() & HAL_RCC_GetPCLK2Freq() tersedia */
#endif

/* ---------- pin ---------- */
#define PIN_PWM_PA    8u
#define PIN_K1_PC     1u
#define PIN_K2_PC     0u
#define PIN_FAN_PE    0u
#define PIN_TRIG_PE   6u
#define PIN_ISR_PE    9u

static volatile uint32_t isr_count = 0, ovr_count = 0, isr_cyc = 0;
static volatile uint16_t temp_code = 2048;      /* sampai konversi pertama */
static uint32_t arr_plus1 = 16800u;
static float    ticks_per_us = 168.0f;
static int      clk_hse = -1;
static const char *rst_cause = "?";
static uint32_t last_isr_seen = 0, last_isr_change_ms = 0;
static int      stall_reported = 0;
static int      iwdg_on = 0;

/* ======================================================================================
 *  Clock: HSE 8 MHz (DevEBox) -> 168 MHz, USB 48 MHz. Cadangan HSI 16 MHz.
 *  Varian STM32duino "Generic F407VGTx" memakai HSI (toleransi +-1 %): frekuensi PWM T1 bisa
 *  meleset dari 10,00 kHz dan USB di luar spesifikasi (+-0,25 %). Fungsi WEAK dari varian ditimpa.
 *  Versi Keil (keil_hal_v2): main() memanggil fungsi ini setelah HAL_Init().
 * ====================================================================================== */
#if defined(QBC_HAL_CLOCK)
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8; osc.PLL.PLLN = 336; osc.PLL.PLLP = RCC_PLLP_DIV2; osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) == HAL_OK) {
        clk_hse = 1;
    } else {
        RCC_OscInitTypeDef h = {0};
        h.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        h.HSIState = RCC_HSI_ON;
        h.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        h.PLL.PLLState = RCC_PLL_ON;
        h.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        h.PLL.PLLM = 16; h.PLL.PLLN = 336; h.PLL.PLLP = RCC_PLLP_DIV2; h.PLL.PLLQ = 7;
        if (HAL_RCC_OscConfig(&h) != HAL_OK) { while (1) { } }
        clk_hse = 0;
    }
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* 168 MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV4;     /*  42 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;     /*  84 MHz -> TIM1 168 MHz, ADC 84/4 = 21 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) { while (1) { } }
}
#endif

/* ======================================================================================
 *  Handler fault inti: matikan PWM & K1 dulu, baru tunggu watchdog.
 *  Tanpa ini, crash membiarkan PWM jalan dengan duty terakhir sampai watchdog reset.
 * ====================================================================================== */
static void emergency_off(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = 0;
    GPIOC->BSRR = (1u << (PIN_K1_PC + 16u));
}
void HardFault_Handler(void)  { emergency_off(); while (1) { } }
void MemManage_Handler(void)  { emergency_off(); while (1) { } }
void BusFault_Handler(void)   { emergency_off(); while (1) { } }
void UsageFault_Handler(void) { emergency_off(); while (1) { } }

/* ======================================================================================
 *  GPIO
 * ====================================================================================== */
static void gpio_out(GPIO_TypeDef *g, uint32_t pin)
{
    g->BSRR = (1u << (pin + 16u));                       /* LOW dulu */
    g->OTYPER &= ~(1u << pin);
    g->OSPEEDR = (g->OSPEEDR & ~(3u << (2u * pin))) | (1u << (2u * pin));
    g->PUPDR &= ~(3u << (2u * pin));
    g->MODER = (g->MODER & ~(3u << (2u * pin))) | (1u << (2u * pin));
}

void hw_safe_gpio_early(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOEEN;
    (void)RCC->AHB1ENR;
    gpio_out(GPIOA, PIN_PWM_PA);     /* PA8 LOW (R26 juga menarik LED TLP250 ke OFF) */
    gpio_out(GPIOC, PIN_K1_PC);
    gpio_out(GPIOC, PIN_K2_PC);
    gpio_out(GPIOE, PIN_FAN_PE);
    gpio_out(GPIOE, PIN_TRIG_PE);
    gpio_out(GPIOE, PIN_ISR_PE);

    /* sebab reset (sekali) */
    {
        uint32_t csr = RCC->CSR;
        if (csr & RCC_CSR_IWDGRSTF)      rst_cause = "WATCHDOG(IWDG)";
        else if (csr & RCC_CSR_SFTRSTF)  rst_cause = "SOFTWARE";
        else if (csr & RCC_CSR_BORRSTF)  rst_cause = "BROWNOUT";
        else if (csr & RCC_CSR_PORRSTF)  rst_cause = "POWER-ON";
        else if (csr & RCC_CSR_PINRSTF)  rst_cause = "PIN-NRST";
        else rst_cause = "LAIN";
        RCC->CSR |= RCC_CSR_RMVF;
    }
}

/* ======================================================================================
 *  TIM1 + ADC1
 * ====================================================================================== */
void hw_init(void)
{
    uint32_t pclk2, timclk, arr;

    /* DWT untuk pengukuran siklus */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* --- TIM1 --- */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;
#if defined(QBC_HAL_CLOCK)
    pclk2 = HAL_RCC_GetPCLK2Freq();
#else
    pclk2 = 84000000u;
#endif
    timclk = ((RCC->CFGR & RCC_CFGR_PPRE2) == 0u) ? pclk2 : 2u * pclk2;
    arr = timclk / 10000u;                               /* 16800 @168 MHz */
    arr_plus1 = arr;
    ticks_per_us = (float)timclk / 1.0e6f;

    TIM1->CR1  = 0;
    TIM1->BDTR = TIM_BDTR_OSSI;                          /* MOE=0 -> keluaran dipaksa idle (LOW) */
    TIM1->PSC  = 0;
    TIM1->ARR  = arr - 1u;
    TIM1->CCR1 = 0;
    TIM1->CCR4 = (uint32_t)(50.0f * ticks_per_us);
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;    /* PWM1: HIGH selama CNT < CCR1 */
    TIM1->CCMR2 = (7u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;    /* PWM2: OC4REF naik di CCR4 */
    TIM1->CCER  = TIM_CCER_CC1E;                         /* CH1 polaritas aktif HIGH; CH4 tanpa pin */
    TIM1->CR2   = (7u << TIM_CR2_MMS_Pos);               /* TRGO = OC4REF; OIS1 = 0 (idle LOW) */
    TIM1->CR1   = TIM_CR1_ARPE;
    TIM1->EGR   = TIM_EGR_UG;
    TIM1->SR    = 0;
    TIM1->CR1  |= TIM_CR1_CEN;

    /* PA8 -> AF1 (TIM1_CH1) setelah timer memaksa LOW */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFu << ((PIN_PWM_PA - 8u) * 4u))) | (1u << ((PIN_PWM_PA - 8u) * 4u));
    GPIOA->OSPEEDR |= (3u << (2u * PIN_PWM_PA));
    GPIOA->MODER = (GPIOA->MODER & ~(3u << (2u * PIN_PWM_PA))) | (2u << (2u * PIN_PWM_PA));

    /* --- ADC1 --- PA2, PA3, PA4, PA5, PA7 analog */
    GPIOA->MODER |= (3u << (2u * 2u)) | (3u << (2u * 3u)) | (3u << (2u * 4u)) | (3u << (2u * 5u)) | (3u << (2u * 7u));
    GPIOA->PUPDR &= ~((3u << (2u * 2u)) | (3u << (2u * 3u)) | (3u << (2u * 4u)) | (3u << (2u * 5u)) | (3u << (2u * 7u)));

    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | (1u << ADC_CCR_ADCPRE_Pos);   /* PCLK2/4 = 21 MHz */
    ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;           /* 12 bit */
    ADC1->CR2 = 0;
    /* 28 siklus untuk kanal 2,3,4,5,7 (sumber: keluaran op-amp + 100 nF -> impedansi rendah) */
    ADC1->SMPR2 = (2u << (3u * 2u)) | (2u << (3u * 3u)) | (2u << (3u * 4u)) | (2u << (3u * 5u)) | (2u << (3u * 7u));
    /* injected: JL = 3 (4 konversi) urutan JSQ1..JSQ4 = IN3 V_OUT, IN5 I_IN, IN4 V_IN, IN2 I_OUT */
    ADC1->JSQR = (3u << ADC_JSQR_JL_Pos) | (3u << ADC_JSQR_JSQ1_Pos) | (5u << ADC_JSQR_JSQ2_Pos) |
                 (4u << ADC_JSQR_JSQ3_Pos) | (2u << ADC_JSQR_JSQ4_Pos);
    /* regular: TEMP (IN7), software start sekali per ISR */
    ADC1->SQR1 = 0;
    ADC1->SQR3 = 7u;
    ADC1->CR2 = ADC_CR2_ADON | (1u << ADC_CR2_JEXTEN_Pos) | (1u << ADC_CR2_JEXTSEL_Pos);  /* TIM1_TRGO, naik */
    { volatile uint32_t i; for (i = 0; i < 2000u; i++) { } }                          /* tSTAB */
    ADC1->SR = 0;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    NVIC_SetPriority(ADC_IRQn, 0);     /* di atas USB (1); SysTick 0 tidak saling preempt */
    NVIC_EnableIRQ(ADC_IRQn);
}

/* ======================================================================================
 *  ISR kontrol
 * ====================================================================================== */
void ADC_IRQHandler(void)
{
    qbc_raw_t raw;
    qbc_out_t out;
    uint32_t t0, ccr4;

    if (!(ADC1->SR & ADC_SR_JEOC)) { ADC1->SR = 0; return; }
    GPIOE->BSRR = (1u << PIN_ISR_PE);
    t0 = DWT->CYCCNT;
    TIM1->SR = ~TIM_SR_UIF;                 /* untuk deteksi overrun di akhir */
    ADC1->SR = ~ADC_SR_JEOC;

    raw.vo   = (uint16_t)ADC1->JDR1;
    raw.iin  = (uint16_t)ADC1->JDR2;
    raw.vin  = (uint16_t)ADC1->JDR3;
    raw.iout = (uint16_t)ADC1->JDR4;
    if (ADC1->SR & ADC_SR_EOC) temp_code = (uint16_t)ADC1->DR;
    raw.temp = temp_code;
    ADC1->CR2 |= ADC_CR2_SWSTART;           /* TEMP berikutnya */

    qbc_isr(&raw, &out);

    /* --- keluaran --- */
    if (!out.pwm_en) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;        /* langsung LOW (OSSI) */
        TIM1->CCR1 = 0;
    } else {
        float d = out.duty;
        if (!(d > 0.0f)) d = 0.0f;
        if (d > 0.95f) d = 0.95f;           /* sabuk pengaman terakhir; batas sebenarnya di qbc_core */
        TIM1->CCR1 = (uint32_t)(d * (float)arr_plus1 + 0.5f);
        TIM1->BDTR |= TIM_BDTR_MOE;
    }
    ccr4 = (uint32_t)(out.t_sample_us * ticks_per_us);
    if (ccr4 < 100u) ccr4 = 100u;
    if (ccr4 > arr_plus1 - 200u) ccr4 = arr_plus1 - 200u;
    TIM1->CCR4 = ccr4;

    GPIOC->BSRR = out.k1 ? (1u << PIN_K1_PC) : (1u << (PIN_K1_PC + 16u));
    GPIOC->BSRR = out.k2 ? (1u << PIN_K2_PC) : (1u << (PIN_K2_PC + 16u));
    GPIOE->BSRR = (out.fan  ? (1u << PIN_FAN_PE)  : (1u << (PIN_FAN_PE + 16u))) |
                  (out.trig ? (1u << PIN_TRIG_PE) : (1u << (PIN_TRIG_PE + 16u)));

    isr_count++;
    if (TIM1->SR & TIM_SR_UIF) ovr_count++;  /* update event terlewati selama ISR */
    isr_cyc = DWT->CYCCNT - t0;
    GPIOE->BSRR = (1u << (PIN_ISR_PE + 16u));
}

/* ======================================================================================
 *  Watchdog & pengawas ISR (dipanggil dari loop)
 * ====================================================================================== */
void hw_iwdg_start(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;     /* debugger berhenti -> watchdog ikut berhenti */
    IWDG->KR  = 0x5555u;
    IWDG->PR  = 3u;                                      /* LSI/32 ~ 1 kHz */
    IWDG->RLR = 100u;                                    /* ~100 ms (LSI 17-47 kHz -> 68-188 ms) */
    IWDG->KR  = 0xAAAAu;
    IWDG->KR  = 0xCCCCu;
    iwdg_on = 1;
}

void hw_kill(void) { emergency_off(); }

void hw_service(uint32_t now_ms)
{
    uint32_t c = isr_count;
    if (c != last_isr_seen) {
        last_isr_seen = c;
        last_isr_change_ms = now_ms;
        stall_reported = 0;
        if (iwdg_on) IWDG->KR = 0xAAAAu;                 /* kick HANYA jika ISR kontrol hidup */
    } else if ((uint32_t)(now_ms - last_isr_change_ms) >= 3u) {
        /* ISR kontrol berhenti >= 3 ms (30 periode): PWM tidak terkendali -> matikan langsung */
        if (TIM1->BDTR & TIM_BDTR_MOE) emergency_off();
        if (!stall_reported) { qbc_external_fault(F_ISR_STALL); stall_reported = 1; }
    }
    qbc_hw_stats(isr_cyc, ovr_count, SystemCoreClock, clk_hse);
}

uint32_t hw_isr_count(void)  { return isr_count; }
uint32_t hw_overruns(void)   { return ovr_count; }
uint32_t hw_isr_cycles(void) { return isr_cyc; }
uint32_t hw_sysclk(void)     { return SystemCoreClock; }
int      hw_clock_is_hse(void) { return clk_hse; }
const char *hw_reset_cause(void) { return rst_cause; }
uint32_t qbc_hw_cycles(void) { return DWT->CYCCNT; }
