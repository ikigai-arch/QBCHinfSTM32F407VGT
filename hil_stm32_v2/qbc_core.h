/* qbc_core.h -- logika inti firmware board QBC v2 (C portabel, tanpa HAL).
 *
 * Pembagian kerja:
 *   qbc_isr()      dipanggil SEKALI tiap Ts = 100 us dari ISR ADC (board) atau dari harness (PC).
 *                  Berisi: konversi sensing, proteksi, mesin status, relai, ramp, pengendali,
 *                  titik cuplik ADC berikutnya. Semua keputusan keselamatan ada di sini.
 *   qbc_command()  dipanggil dari loop (konteks non-ISR) untuk satu baris perintah teks.
 *                  Perintah yang mengubah status dimasukkan ke antrean dan dieksekusi oleh ISR.
 *   qbc_poll_*()   dipanggil dari loop untuk mengambil teks keluaran (event, telemetri, dump).
 *
 * Konkruensi: antrean perintah dan event berupa ring buffer satu-produsen-satu-konsumen.
 * Konfigurasi skalar (float 32-bit, selaras) ditulis langsung dari loop (atomik di Cortex-M4).
 */
#ifndef QBC_CORE_H
#define QBC_CORE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t vo, iin, vin, iout, temp;   /* kode ADC 12 bit */
} qbc_raw_t;

typedef struct {
    float    duty;          /* duty untuk periode PWM berikutnya (0..1)            */
    uint8_t  pwm_en;        /* 1 = output PWM aktif (MOE), 0 = paksa LOW           */
    uint8_t  k1, k2, fan;   /* level pin PC1, PC0, PE0                             */
    uint8_t  trig;          /* level pin PE6                                       */
    float    t_sample_us;   /* saat cuplik ADC di periode berikutnya (CCR4)         */
} qbc_out_t;

enum qbc_state {
    ST_OFF = 0, ST_ARMING = 1, ST_ARMED = 2, ST_RUN_OL = 3, ST_RUN_CL = 4,
    ST_PWMTEST = 5, ST_DISARM = 6, ST_FAULT = 7
};

/* bit fault (latched) */
#define F_OVP        (1u << 0)
#define F_OCP_IN_F   (1u << 1)
#define F_OCP_IN_S   (1u << 2)
#define F_OCP_OUT_F  (1u << 3)
#define F_OCP_OUT_S  (1u << 4)
#define F_UV_IN      (1u << 5)
#define F_OV_IN      (1u << 6)
#define F_OTP        (1u << 7)
#define F_NTC        (1u << 8)
#define F_VO_PLAUS   (1u << 9)
#define F_DSAT       (1u << 10)
#define F_NOLOAD     (1u << 11)
#define F_ARM        (1u << 12)
#define F_USER       (1u << 13)
#define F_ISR_STALL  (1u << 14)
#define F_SIM_REAL   (1u << 15)
#define F_ADC_SAT    (1u << 16)
#define F_RELAY      (1u << 17)
#define F_NFAULT     18

void        qbc_init(void);
void        qbc_isr(const qbc_raw_t *raw, qbc_out_t *out);
int         qbc_command(const char *line, char *reply, int replylen);
void        qbc_set_link(int connected);             /* status DTR USB, dari loop */
void        qbc_external_fault(uint32_t fmask);      /* dari lapisan HW (mis. ISR macet) */
void        qbc_hw_stats(uint32_t isr_cycles, uint32_t overruns, uint32_t sysclk_hz, int hse);

int         qbc_poll_event(char *buf, int n);        /* 0 jika tidak ada */
int         qbc_poll_telemetry(uint32_t now_ms, char *buf, int n);
int         qbc_poll_output(char *buf, int n);       /* hasil raw / dump capture */

/* akses baca untuk harness & status */
int         qbc_state(void);
uint32_t    qbc_faults(void);
float       qbc_vref_now(void);
float       qbc_duty(void);
const char *qbc_state_name(int s);
const char *qbc_fault_name(int bit);
uint32_t    qbc_ticks(void);

#ifdef __cplusplus
}
#endif
#endif
