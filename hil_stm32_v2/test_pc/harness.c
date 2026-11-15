/* harness.c -- uji firmware QBC v2 tanpa board (C biasa, gcc/MinGW).
 *
 * Yang diuji: qbc_core.c (mesin status, proteksi, relai, ramp, perintah, logging), ctrl_mux.c
 * (H-inf / PI / MPSO-PI + anti-windup), file pengendali asli, dan qbc_sim.c. File-file itu
 * dikompilasi APA ADANYA dari folder sketch di atasnya (..) (tidak ada #ifdef khusus PC di dalamnya).
 * Yang TIDAK diuji: hw_f407.c (register TIM1/ADC/IWDG) dan hil_stm32_v2.ino (USB) -> butuh board.
 *
 * Model "board" di harness:
 *   - plant rata-rata QBC 8 parasitik (qbc_sim.c, set 'hw' bawaan; set 'model' untuk pembanding),
 *     10 sub-langkah Euler semi-implisit per Ts (akurasi dicek di uji plant_acc);
 *   - relai K1 (sumber -> converter) dan K2 (converter -> beban), dikendalikan out.k1/out.k2;
*   - sensor seperti di skematik: V_IN sebelum K1, V_OUT dan I_OUT SETELAH K2, I_IN = iL1;
 *     kuantisasi 12 bit dengan skala qbc_config.h + derau +-1 LSB; NTC 10k/B3950 dengan R11 10k;
 *   - urutan per tick: cuplik -> qbc_isr() -> duty baru berlaku selama Ts berikutnya
 *     (= unit delay di MIL-2/PIL).
 *
 * Setiap uji dijalankan di proses terpisah supaya state statis firmware selalu bersih, seperti setelah
 * reset board. Pemakaian:  ./harness [nama_uji ...]   (tanpa argumen = semua)
 * Keluaran: ringkasan di stdout, rincian di hasil/hasil_uji.txt, jejak CSV di folder hasil.
 */
#define _POSIX_C_SOURCE 200809L   /* popen/pclose */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
#include "qbc_core.h"
#include "qbc_config.h"
#include "qbc_sim.h"
#include "ctrl_mux.h"

uint32_t qbc_hw_cycles(void) { return 0; }

/* ======================================================================================
 *  Model board
 * ====================================================================================== */
typedef struct {
    qbc_plant_par_t p;
    qbc_plant_x_t   x;
    float vsrc;          /* sumber lab di depan K1 (V)                 */
    float R;             /* beban di belakang K2 (Ohm), <= 0 = terbuka  */
    float temp_c;        /* suhu NTC (C)                               */
    int   ntc_open;
    int   k2_stuck_open; /* kontak K2 tidak menutup walau kumparan ON   */
    int   k1_welded;     /* kontak K1 lengket tertutup                  */
    float vo_ext;        /* tegangan dari luar di sisi beban (mis. baterai) */
    int   noise;
    int   link;
    float vterm;
    qbc_out_t out;
    /* statistik */
    float vc2_max, il1_max, il2_max, vterm_max;
    uint32_t ticks;
} board_t;

static board_t B;
static FILE *flog;

static uint16_t code_of(float x, float a, float b)
{
    float c = (x - b) / a;
    if (B.noise) c += (float)(rand() % 3 - 1);
    if (c < 0) c = 0;
    if (c > 4095) c = 4095;
    return (uint16_t)(c + 0.5f);
}

static uint16_t ntc_code(void)
{
    float r;
    if (B.ntc_open) return 0;
    r = NTC_R0_OHM * expf(NTC_BETA * (1.0f / (B.temp_c + 273.15f) - 1.0f / NTC_T0_K));
    return (uint16_t)(4095.0f * NTC_RPULL_OHM / (r + NTC_RPULL_OHM) + 0.5f);
}

static void board_reset(int hw)
{
    memset(&B, 0, sizeof(B));
    if (hw) qbc_plant_par_hw(&B.p); else qbc_plant_par_model(&B.p);
    qbc_plant_reset(&B.x);
    B.vsrc = 18.0f; B.R = 135.0f; B.temp_c = 25.0f; B.noise = 1; B.link = 1;
}

/* satu Ts */
static void tick1(void)
{
    qbc_raw_t raw;
    int k1 = B.out.k1 || B.k1_welded;
    int k2 = B.out.k2 && !B.k2_stuck_open;
    float vside = k2 ? B.vterm : B.vo_ext;
    float io = (k2 && B.R > 0.0f) ? B.vterm / B.R : 0.0f;
    int i;
    float d, vin_eff, Reff;

    raw.vo   = code_of(vside, CAL_VO_A, CAL_VO_B);
    raw.vin  = code_of(B.vsrc, CAL_VIN_A, CAL_VIN_B);
    raw.iin  = code_of(B.x.iL1, CAL_IIN_A, CAL_IIN_B);
    raw.iout = code_of(io, CAL_IOUT_A, CAL_IOUT_B);
    raw.temp = ntc_code();
    qbc_set_link(B.link);
    qbc_isr(&raw, &B.out);

    d = B.out.pwm_en ? B.out.duty : 0.0f;
    if (d > 0.95f) d = 0.95f;                 /* sabuk pengaman hw_f407.c */
    k1 = B.out.k1 || B.k1_welded;
    k2 = B.out.k2 && !B.k2_stuck_open;
    vin_eff = k1 ? B.vsrc : 0.0f;
    Reff = k2 ? B.R : -1.0f;
    for (i = 0; i < 10; i++) B.vterm = qbc_plant_step(&B.p, &B.x, d, vin_eff, Reff, QBC_TS / 10.0f);
    if (B.x.vC2 > B.vc2_max) B.vc2_max = B.x.vC2;
    if (B.vterm > B.vterm_max) B.vterm_max = B.vterm;
    if (B.x.iL1 > B.il1_max) B.il1_max = B.x.iL1;
    if (B.x.iL2 > B.il2_max) B.il2_max = B.x.iL2;
    B.ticks++;
}

/* ======================================================================================
 *  Event & perintah
 * ====================================================================================== */
static char evlog[200000];
static size_t evlen = 0;

static void drain(void)
{
    char b[300];
    int n;
    while ((n = qbc_poll_event(b, sizeof(b))) > 0) {
        if (evlen + (size_t)n < sizeof(evlog) - 1) { memcpy(evlog + evlen, b, (size_t)n); evlen += (size_t)n; evlog[evlen] = 0; }
        fputs("      ", flog); fputs(b, flog);
    }
}
static int ev_seen(const char *s) { return strstr(evlog, s) != NULL; }
static void ev_clear(void) { evlen = 0; evlog[0] = 0; }

static char lastrep[2000];
static void cmd(const char *c)
{
    qbc_command(c, lastrep, sizeof(lastrep));
    fprintf(flog, "    > %-22s %s", c, lastrep[0] ? lastrep : "\n");
    if (!lastrep[0] || lastrep[strlen(lastrep) - 1] != '\n') fputc('\n', flog);
}

/* invarian keselamatan yang dicek SETIAP tick */
static int inv_fail = 0;
static char inv_msg[200];
static void check_inv(void)
{
    int s = qbc_state();
    const qbc_out_t *o = &B.out;
    if (inv_fail) return;
    if ((s == ST_OFF || s == ST_ARMING || s == ST_ARMED || s == ST_DISARM || s == ST_FAULT) && o->pwm_en)
        { inv_fail = 1; snprintf(inv_msg, sizeof inv_msg, "PWM aktif di status %s", qbc_state_name(s)); }
    if (s == ST_OFF && (o->k1 || o->k2))
        { inv_fail = 1; snprintf(inv_msg, sizeof inv_msg, "relai tertutup di OFF"); }
    if (s == ST_PWMTEST && (o->k1 || o->k2))
        { inv_fail = 1; snprintf(inv_msg, sizeof inv_msg, "relai tertutup di PWMTEST"); }
    if (o->pwm_en && s == ST_RUN_CL && (o->duty < DUTY_CL_MIN - 1e-6f || o->duty > DUTY_CL_MAX + 1e-6f))
        { inv_fail = 1; snprintf(inv_msg, sizeof inv_msg, "duty CL %.4f di luar 0,05..0,85", o->duty); }
    if (o->pwm_en && s == ST_RUN_OL && o->duty > OL_DMAX_HARD + 1e-6f)
        { inv_fail = 1; snprintf(inv_msg, sizeof inv_msg, "duty OL %.4f > batas keras", o->duty); }
    if (o->k2 && !o->k1 && s != ST_DISARM && s != ST_FAULT)
        { inv_fail = 1; snprintf(inv_msg, sizeof inv_msg, "K2 tertutup tanpa K1 di %s", qbc_state_name(s)); }
    if (!(o->duty == o->duty))
        { inv_fail = 1; snprintf(inv_msg, sizeof inv_msg, "duty NaN"); }
}

/* jejak CSV (opsional) */
static FILE *ftr = NULL;
static uint32_t tr_dec = 10;
static void trace_open(const char *name, uint32_t dec)
{
    char p[200];
    snprintf(p, sizeof p, "hasil/%s.csv", name);
    ftr = fopen(p, "w");
    tr_dec = dec;
    if (ftr) fprintf(ftr, "t_ms,state,vref,vo_term,vo_sensor_k2,vin,il1,il2,iout,duty,pwm,k1,k2\n");
}
static void trace_close(void) { if (ftr) fclose(ftr); ftr = NULL; }

/* jalankan ms milidetik; fn opsional dipanggil tiap tick */
typedef void (*hook_t)(void);
static void run_ms(float ms, hook_t fn)
{
    uint32_t n = (uint32_t)(ms * QBC_TICKS_PER_MS + 0.5f), i;
    for (i = 0; i < n; i++) {
        tick1();
        check_inv();
        if (fn) fn();
        if ((B.ticks & 15u) == 0) drain();
        if (ftr && (B.ticks % tr_dec) == 0) {
            int k2 = B.out.k2 && !B.k2_stuck_open;
            fprintf(ftr, "%.2f,%s,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.5f,%d,%d,%d\n",
                    B.ticks * 0.1f, qbc_state_name(qbc_state()), qbc_vref_now(), B.vterm, k2 ? B.vterm : B.vo_ext,
                    B.vsrc, B.x.iL1, B.x.iL2, (k2 && B.R > 0) ? B.vterm / B.R : 0.0f, B.out.duty, B.out.pwm_en,
                    B.out.k1, B.out.k2);
        }
    }
    drain();
}

/* jalankan sampai kondisi atau batas waktu; kembalikan ms yang dipakai (-1 jika timeout) */
typedef int (*cond_t)(void);
static float run_until(cond_t c, float max_ms)
{
    uint32_t n = (uint32_t)(max_ms * QBC_TICKS_PER_MS), i;
    for (i = 0; i < n; i++) {
        tick1(); check_inv();
        if ((B.ticks & 15u) == 0) drain();
        if (c()) { drain(); return (float)(i + 1) * 0.1f; }
    }
    drain();
    return -1.0f;
}
static int c_armed(void)  { return qbc_state() == ST_ARMED; }
static int c_fault(void)  { return qbc_state() == ST_FAULT; }
static int c_off(void)    { return qbc_state() == ST_OFF; }
static int c_pwmoff(void) { return !B.out.pwm_en; }

/* ======================================================================================
 *  Pencatatan hasil
 * ====================================================================================== */
static int n_chk = 0, n_fail = 0;
static char summary[4000];
static void CHECK(int ok, const char *fmt, ...)
{
    va_list ap;
    char m[400];
    va_start(ap, fmt); vsnprintf(m, sizeof m, fmt, ap); va_end(ap);
    n_chk++;
    if (!ok) n_fail++;
    fprintf(flog, "  [%s] %s\n", ok ? "LOLOS" : "GAGAL", m);
    if (!ok) { strncat(summary, "    GAGAL: ", sizeof(summary) - strlen(summary) - 1); strncat(summary, m, sizeof(summary) - strlen(summary) - 1); strncat(summary, "\n", sizeof(summary) - strlen(summary) - 1); }
}
static void NOTE(const char *fmt, ...)
{
    va_list ap;
    char m[400];
    va_start(ap, fmt); vsnprintf(m, sizeof m, fmt, ap); va_end(ap);
    fprintf(flog, "  [INFO ] %s\n", m);
    strncat(summary, "    info: ", sizeof(summary) - strlen(summary) - 1);
    strncat(summary, m, sizeof(summary) - strlen(summary) - 1);
    strncat(summary, "\n", sizeof(summary) - strlen(summary) - 1);
}

/* ======================================================================================
 *  Rangkaian pembantu
 * ====================================================================================== */
static void boot(int hw)
{
    board_reset(hw);
    qbc_init();
    ev_clear();
    run_ms(5, NULL);
}

static int arm_ok(void)
{
    cmd("arm");
    return run_until(c_armed, 1000) > 0;
}

/* statistik jendela */
typedef struct { double vo_sum, vo_min, vo_max, d_sum, il1_sum, il1_max; uint32_t n; } win_t;
static win_t W;
static void win_hook(void)
{
    if (W.n == 0) { W.vo_min = W.vo_max = B.vterm; W.il1_max = B.x.iL1; }
    W.vo_sum += B.vterm; W.d_sum += B.out.duty; W.il1_sum += B.x.iL1; W.n++;
    if (B.vterm < W.vo_min) W.vo_min = B.vterm;
    if (B.vterm > W.vo_max) W.vo_max = B.vterm;
    if (B.x.iL1 > W.il1_max) W.il1_max = B.x.iL1;
}
static void win_run(float ms) { memset(&W, 0, sizeof W); run_ms(ms, win_hook); }

/* waktu sampai Vo masuk +-2 % dan tetap di sana */
static float settle_ms(float target, float band, float max_ms)
{
    uint32_t n = (uint32_t)(max_ms * QBC_TICKS_PER_MS), i, last_out = 0;
    for (i = 0; i < n; i++) {
        tick1(); check_inv();
        if ((B.ticks & 15u) == 0) drain();
        if (ftr && (B.ticks % tr_dec) == 0) {
            fprintf(ftr, "%.2f,%s,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.5f,%d,%d,%d\n",
                    B.ticks * 0.1f, qbc_state_name(qbc_state()), qbc_vref_now(), B.vterm, B.vterm, B.vsrc,
                    B.x.iL1, B.x.iL2, B.R > 0 ? B.vterm / B.R : 0.0f, B.out.duty, B.out.pwm_en, B.out.k1, B.out.k2);
        }
        if (fabsf(B.vterm - target) > band) last_out = i + 1;
    }
    drain();
    return (last_out >= n) ? -1.0f : (float)last_out * 0.1f;
}

/* start closed-loop sampai 150 V tunak; kembalikan 1 jika sukses */
static int start_cl(const char *ctrl, float vref)
{
    char c[40];
    snprintf(c, sizeof c, "ctrl %s", ctrl); cmd(c);
    run_ms(1, NULL);
    if (!arm_ok()) return 0;
    snprintf(c, sizeof c, "v %.1f", vref); cmd(c);
    cmd("cl");
    run_ms(3000, NULL);                                  /* PI-FR lambat: settle ~1,5 s */
    return qbc_state() == ST_RUN_CL && fabsf(B.vterm - vref) < 1.0f;
}

/* ======================================================================================
 *  UJI
 * ====================================================================================== */
static void t_boot_safe(void)
{
    int bad = 0;
    uint32_t i;
    board_reset(1);
    /* sebelum qbc_init: keluaran default (struct nol) */
    qbc_init();
    for (i = 0; i < 20000; i++) {       /* 2 s */
        tick1(); check_inv();
        if (B.out.pwm_en || B.out.k1 || B.out.k2 || B.out.duty != 0.0f) bad++;
    }
    drain();
    CHECK(qbc_state() == ST_OFF, "status setelah boot = %s (harus OFF)", qbc_state_name(qbc_state()));
    CHECK(bad == 0, "2 s setelah boot dengan Vin 18 V: PWM/K1/K2 tidak pernah aktif (%d tick salah)", bad);
    CHECK(B.vc2_max < 0.01f, "Vo tetap 0 (maks %.3f V)", B.vc2_max);
    CHECK(qbc_faults() == 0, "tidak ada fault palsu saat idle (faults=0x%x)", qbc_faults());
    CHECK(B.out.fan == 0, "kipas mati saat idle 25 C");
}

static void t_refuse(void)
{
    boot(1);
    cmd("ol"); cmd("cl"); cmd("d 0.5"); cmd("s 5");
    run_ms(10, NULL);
    CHECK(qbc_state() == ST_OFF && !B.out.pwm_en, "ol/cl/d/s di OFF tidak menyalakan PWM");
    CHECK(ev_seen("TOLAK,status_tidak_sesuai"), "event TOLAK tercatat");
    ev_clear();
    B.vsrc = 3.0f; run_ms(1, NULL); cmd("arm"); run_ms(5, NULL);
    CHECK(qbc_state() == ST_OFF && ev_seen("Vin_di_luar_jendela"), "arm ditolak saat Vin 3 V");
    ev_clear();
    B.vsrc = 24.0f; run_ms(1, NULL); cmd("arm"); run_ms(5, NULL);
    CHECK(qbc_state() == ST_OFF && ev_seen("Vin_di_luar_jendela"), "arm ditolak saat Vin 24 V (> 22 V)");
    ev_clear();
    B.vsrc = 18.0f; B.vo_ext = 12.0f; run_ms(1, NULL); cmd("arm"); run_ms(5, NULL);
    CHECK(qbc_state() == ST_OFF && ev_seen("ada_tegangan_di_sisi_beban"), "arm ditolak saat ada 12 V di sisi beban");
    B.vo_ext = 0.0f;
    ev_clear();
    cmd("arm"); run_ms(2, NULL); cmd("cl"); run_ms(2, NULL);
    CHECK(qbc_state() == ST_ARMING && !B.out.pwm_en, "cl saat ARMING ditolak");
    run_until(c_armed, 1000);
    cmd("v 0"); cmd("cl"); run_ms(2, NULL);
    CHECK(qbc_state() == ST_ARMED && ev_seen("vref_belum_diset"), "cl dengan vref 0 ditolak");
    cmd("v 200"); run_ms(1, NULL);
    CHECK(strstr(lastrep, "ERR") != NULL, "v 200 ditolak di parser (batas 160 V)");
    cmd("dmax 0.9");
    CHECK(strstr(lastrep, "ERR") != NULL, "dmax 0.9 ditolak (batas keras 0,75)");
    cmd("lim ovp 190");
    CHECK(strstr(lastrep, "ERR") != NULL, "lim ovp 190 ditolak (batas keras 185 V)");
    cmd("lim iinf 25");
    CHECK(strstr(lastrep, "ERR") != NULL, "lim iinf 25 ditolak (batas keras 21 A)");
}

static uint32_t k1_tick, k2_tick;
static void hook_relay(void)
{
    if (B.out.k1 && !k1_tick) k1_tick = B.ticks;
    if (B.out.k2 && !k2_tick) k2_tick = B.ticks;
}
static void t_arm_seq(void)
{
    uint32_t t0;
    boot(1);
    k1_tick = k2_tick = 0;
    t0 = B.ticks;
    cmd("arm");
    run_ms(600, hook_relay);
    CHECK(qbc_state() == ST_ARMED, "status ARMED setelah arm (sekarang %s)", qbc_state_name(qbc_state()));
    CHECK(k1_tick && (k1_tick - t0) <= 2, "K1 tertutup pada tick pertama (%u)", k1_tick - t0);
    CHECK(k2_tick && (k2_tick - k1_tick) >= K1_SETTLE_MS * 10, "K2 baru tertutup %.1f ms setelah K1 (>= %u ms)",
          (k2_tick - k1_tick) * 0.1f, K1_SETTLE_MS);
    CHECK(!B.out.pwm_en, "PWM tetap mati di ARMED");
    NOTE("Vo tanpa switching setelah arm = %.2f V (Vin 18 V - dioda), iL1 puncak saat K1 menutup = %.2f A",
         B.vterm, B.il1_max);
    CHECK(B.il1_max < IIN_FAST_DEFAULT, "arus inrush saat K1 menutup (%.1f A) < OCP cepat", B.il1_max);
    cmd("disarm");
    run_until(c_off, 6000);
    CHECK(qbc_state() == ST_OFF && !B.out.k1 && !B.out.k2, "disarm -> OFF, relai terbuka");
}

static void ramp_one(const char *ctrl, int hw)
{
    char nm[64];
    float st_ms, il1_ss, d_ss;
    boot(hw);
    snprintf(nm, sizeof nm, "ramp150_%s_%s", ctrl, hw ? "hw" : "model");
    trace_open(nm, 10);
    { char c[30]; snprintf(c, sizeof c, "ctrl %s", ctrl); cmd(c); }
    run_ms(1, NULL);
    if (!arm_ok()) { CHECK(0, "%s: arm gagal", nm); trace_close(); return; }
    B.il1_max = 0; B.vterm_max = 0;
    cmd("v 150"); cmd("cl");
    st_ms = settle_ms(150.0f, 3.0f, 2000);              /* +-2 % */
    trace_close();
    win_run(200);
    il1_ss = (float)(W.il1_sum / W.n); d_ss = (float)(W.d_sum / W.n);
    CHECK(qbc_state() == ST_RUN_CL && qbc_faults() == 0, "%s: tetap RUN_CL tanpa fault (state %s, faults 0x%x)",
          nm, qbc_state_name(qbc_state()), qbc_faults());
    CHECK(fabs(W.vo_sum / W.n - 150.0) < 0.5, "%s: Vo rata-rata %.2f V (150 +- 0,5)", nm, W.vo_sum / W.n);
    CHECK(st_ms > 0, "%s: masuk +-2 %% dalam %.0f ms (ramp 300 V/s = 445 ms)", nm, st_ms);
    CHECK(B.vterm_max < 156.0f, "%s: Vo puncak %.2f V < 156 V", nm, B.vterm_max);
    CHECK(B.il1_max < IIN_FAST_DEFAULT, "%s: iL1 puncak %.2f A < OCP cepat %.0f A", nm, B.il1_max, IIN_FAST_DEFAULT);
    NOTE("%s: D tunak %.4f, iL1 tunak %.2f A, iL1 puncak %.2f A, riak Vo %.2f Vpp (rata-rata), settle %.0f ms",
         nm, d_ss, il1_ss, B.il1_max, W.vo_max - W.vo_min, st_ms);
}
static void t_ramp150(void)
{
    ramp_one("hinf", 1); ramp_one("pi", 1); ramp_one("mpso", 1);
    ramp_one("hinf", 0); ramp_one("pi", 0); ramp_one("mpso", 0);
}

static void s2_case(const char *ctrl, float R2, const char *tag)
{
    char nm[60];
    float pk, mn;
    boot(1);
    if (!start_cl(ctrl, 150)) { CHECK(0, "%s: start gagal", ctrl); return; }
    snprintf(nm, sizeof nm, "%s_%s_hw", tag, ctrl);
    trace_open(nm, 5);
    run_ms(20, NULL);
    B.R = R2; B.vterm_max = 0; B.il1_max = 0;
    win_run(500);
    pk = B.vterm_max; mn = (float)W.vo_min;
    win_run(100);
    if (qbc_faults() == 0)
        NOTE("%s %s: 135 -> %.0f Ohm: Vo %.2f..%.2f V, D tunak %.4f, iL1 puncak %.2f A", tag, ctrl, R2, mn, pk,
             (float)(W.d_sum / W.n), B.il1_max);
    else
        NOTE("%s %s: 135 -> %.0f Ohm: TRIP %s%s (iL1 puncak %.2f A)", tag, ctrl, R2,
             (qbc_faults() & F_OCP_IN_F) ? "OCP_IN_F " : "", (qbc_faults() & F_OCP_IN_S) ? "OCP_IN_S" : "", B.il1_max);
    if (R2 > 135.0f) {
        CHECK(qbc_faults() == 0, "%s %s: 135 -> %.0f Ohm tanpa trip (faults 0x%x)", tag, ctrl, R2, qbc_faults());
        CHECK(fabs(W.vo_sum / W.n - 150) < 0.6, "%s %s: Vo kembali ke %.2f V", tag, ctrl, W.vo_sum / W.n);
        CHECK(pk < OVP_V_DEFAULT, "%s %s: overshoot %.2f V di bawah OVP", tag, ctrl, pk);
        B.R = 135.0f; B.vterm_max = 0; B.il1_max = 0;
        win_run(600);
        CHECK(qbc_faults() == 0, "%s %s: %.0f -> 135 Ohm tanpa trip, Vo min %.2f V, iL1 puncak %.2f A", tag, ctrl, R2, W.vo_min, B.il1_max);
    }
    trace_close();
}
static void t_load_step(void)
{
    s2_case("hinf", 270.0f, "S2"); s2_case("pi", 270.0f, "S2"); s2_case("mpso", 270.0f, "S2");
}
static void t_s2_mil(void)
{
    s2_case("hinf", 101.0f, "S2mil"); s2_case("pi", 101.0f, "S2mil"); s2_case("mpso", 101.0f, "S2mil");
    CHECK(1, "skenario MIL 135 -> 101 Ohm dicatat saja (lihat info)");
}

static int trig_hi_ticks;
static void hook_trig(void) { if (B.out.trig) trig_hi_ticks++; }
static void t_vref_step(void)
{
    const char *C[3] = { "hinf", "pi", "mpso" };
    int c;
    for (c = 0; c < 3; c++) {
        char nm[60];
        boot(1);
        if (!start_cl(C[c], 150)) { CHECK(0, "%s: start gagal", C[c]); continue; }
        snprintf(nm, sizeof nm, "S1_vref_%s_hw", C[c]);
        trace_open(nm, 5);
        ev_clear(); trig_hi_ticks = 0;
        cmd("s 5");
        memset(&W, 0, sizeof W);
        run_ms(500, hook_trig);
        win_run(100);
        trace_close();
        CHECK(fabs(W.vo_sum / W.n - 155) < 0.6, "S1 %s: Vo -> %.2f V setelah step +5 V", C[c], W.vo_sum / W.n);
        CHECK(trig_hi_ticks == 10, "S1 %s: pin TRIG (PE6) HIGH selama %d tick = %.1f ms (1 ms)", C[c], trig_hi_ticks, trig_hi_ticks * 0.1f);
        CHECK(ev_seen("INFO,STEP,155"), "S1 %s: event STEP tercatat", C[c]);
        CHECK(qbc_faults() == 0, "S1 %s: tanpa fault", C[c]);
    }
}

static void t_vin_step(void)
{
    const char *C[3] = { "hinf", "pi", "mpso" };
    int c, dir;
    for (c = 0; c < 3; c++) for (dir = 0; dir < 2; dir++) {
        float v2 = dir ? 16.2f : 19.8f, pk;
        char nm[60];
        boot(1);
        if (!start_cl(C[c], 150)) { CHECK(0, "%s: start gagal", C[c]); continue; }
        snprintf(nm, sizeof nm, "S3_vin%s_%s_hw", dir ? "turun" : "naik", C[c]);
        trace_open(nm, 5);
        B.vterm_max = 0; B.il1_max = 0;
        B.vsrc = v2; win_run(800);
        pk = B.vterm_max;
        CHECK(qbc_faults() == 0, "S3 %s: Vin 18 -> %.1f V tanpa trip (faults 0x%x)", C[c], v2, qbc_faults());
        if (dir == 0) NOTE("S3 %s: Vin +10 %%: Vo puncak %.2f V (OVP %.0f V; v1 165 V)", C[c], pk, OVP_V_DEFAULT);
        else NOTE("S3 %s: Vin -10 %%: Vo minimum %.2f V, D tunak %.4f, iL1 puncak %.2f A", C[c], W.vo_min, (float)(W.d_sum / W.n), B.il1_max);
        B.vsrc = 18.0f; B.il1_max = 0; run_ms(1200, NULL); win_run(300);
        CHECK(qbc_faults() == 0 && fabs(W.vo_sum / W.n - 150) < 0.6, "S3 %s: Vin kembali %.1f -> 18 V tanpa trip, Vo %.2f V, iL1 puncak %.2f A",
              C[c], v2, W.vo_sum / W.n, B.il1_max);
        trace_close();
    }
}

/* S3 dengan slew PSU: Vin 16,2 -> 18 V dalam 0 / 1 / 10 ms, iL1 puncak H-inf */
static void t_vin_slew(void)
{
    float ramps[3] = { 0.0f, 1.0f, 10.0f };
    int r;
    for (r = 0; r < 3; r++) {
        uint32_t i, n = (uint32_t)(ramps[r] * 10.0f);
        boot(1);
        if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
        B.vsrc = 16.2f; run_ms(1500, NULL);
        B.il1_max = 0;
        for (i = 0; i < n; i++) { B.vsrc = 16.2f + 1.8f * (float)(i + 1) / (float)n; run_ms(0.1f, NULL); }
        B.vsrc = 18.0f; run_ms(500, NULL);
        NOTE("H-inf Vin 16,2 -> 18 V dengan ramp %.0f ms: iL1 puncak %.2f A, faults 0x%x", ramps[r], B.il1_max, qbc_faults());
    }
    CHECK(1, "dicatat saja");
}

static uint32_t t_first_over = 0, t_pwm_off = 0;
static float ovp_thr = 0;
static void hook_ovp(void)
{
    int k2 = B.out.k2 && !B.k2_stuck_open;
    float vs = k2 ? B.vterm : 0;
    if (!t_first_over && vs > ovp_thr) t_first_over = B.ticks;
    if (t_first_over && !t_pwm_off && !B.out.pwm_en) t_pwm_off = B.ticks;
}
static uint32_t t_k1open, t_k2open; static float vo_k2open;
static void hook_open(void)
{
    if (!B.out.k1 && !t_k1open) t_k1open = B.ticks;
    if (!B.out.k2 && !t_k2open) { t_k2open = B.ticks; vo_k2open = B.vterm; }
}
static void hook_ovp_open(void) { hook_ovp(); if (t_pwm_off) hook_open(); }
static void t_ovp(void)
{
    uint32_t t0;
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    cmd("lim ovp 152");
    run_ms(50, NULL);
    CHECK(qbc_faults() == 0, "OVP 152 V belum trip di 150 V (riak < 2 V)");
    ovp_thr = 152; t_first_over = t_pwm_off = 0;
    trace_open("U10_ovp_hinf_hw", 1);
    t_k1open = t_k2open = 0;
    cmd("s 5");
    run_ms(100, hook_ovp_open);                          /* 100 us/sampel di sekitar trip */
    trace_close();
    run_ms(5900, hook_ovp_open);
    CHECK(qbc_faults() & F_OVP, "trip OVP saat Vo melewati 152 V (faults 0x%x)", qbc_faults());
    CHECK(t_pwm_off && (t_pwm_off - t_first_over) <= OVP_N + 1,
          "PWM mati %u tick setelah Vo > ambang (<= %u)", t_pwm_off - t_first_over, OVP_N + 1);
    CHECK(ev_seen("FAULT,OVP"), "event FAULT,OVP tercatat");
    NOTE("Vo puncak selama uji OVP = %.2f V", B.vterm_max);
    t0 = t_pwm_off;
    CHECK(t_k1open && (t_k1open - t0) <= (K1_OPEN_DELAY_MS + 1) * 10, "K1 terbuka %.1f ms setelah trip", (t_k1open - t0) * 0.1f);
    CHECK(t_k2open > t_k1open, "K2 terbuka setelah K1 (%.1f ms setelah trip, Vo saat K2 dibuka %.1f V)", (t_k2open - t0) * 0.1f, vo_k2open);
    CHECK(vo_k2open < K2_OPEN_VO_V + 1, "K2 dibuka saat Vo < 30 V (tidak memutus 150 V DC)");
    cmd("ol"); cmd("arm"); run_ms(5, NULL);
    CHECK(qbc_state() == ST_FAULT && !B.out.pwm_en, "fault latched: arm/ol ditolak sebelum clr");
    cmd("clr"); run_ms(2, NULL);
    CHECK(qbc_state() == ST_OFF && qbc_faults() == 0, "clr -> OFF");
    cmd("limreset");
}

static void t_ovp_clr_early(void)
{
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    cmd("!");
    run_ms(1, NULL);
    CHECK(!B.out.pwm_en && qbc_state() == ST_FAULT, "'!' -> PWM mati pada tick berikutnya, FAULT USER");
    cmd("clr"); run_ms(2, NULL);
    CHECK(qbc_state() == ST_FAULT && ev_seen("relai_belum_selesai_dibuka"), "clr ditolak selama relai belum terbuka");
    run_until(c_off, 1); /* no-op */
    run_ms(6000, NULL);
    cmd("clr"); run_ms(2, NULL);
    CHECK(qbc_state() == ST_OFF, "clr diterima setelah urutan buka relai selesai");
}

static void t_ocp_out(void)
{
    float ms;
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    B.il1_max = 0; B.il2_max = 0;
    trace_open("U10_short_out_hinf_hw", 1);
    B.R = 5.0f;                                          /* hubung singkat parsial di keluaran */
    ms = run_until(c_fault, 100);
    run_ms(30, NULL);
    trace_close();
    CHECK(ms > 0 && (qbc_faults() & (F_OCP_OUT_F | F_OCP_IN_F)), "beban 5 Ohm -> trip dalam %.1f ms (faults 0x%x)", ms, qbc_faults());
    NOTE("hubung singkat keluaran: fault pertama %s, iL1 puncak %.2f A, iL2 puncak %.2f A",
         qbc_faults() & F_OCP_OUT_F ? "OCP_OUT_F" : "lain", B.il1_max, B.il2_max);
}

static void t_ocp_in(void)
{
    float ms;
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    cmd("lim iinf 8");
    ms = run_until(c_fault, 20);
    CHECK(ms > 0 && (qbc_faults() & F_OCP_IN_F), "lim iinf 8 A (iL1 ~12,6 A) -> OCP_IN_F dalam %.1f ms", ms);
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    cmd("lim iins 11");
    ms = run_until(c_fault, 3000);
    CHECK(ms > 0 && (qbc_faults() & F_OCP_IN_S), "lim iins 11 A -> OCP_IN_S (rata-rata lambat) setelah %.0f ms", ms);
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    cmd("lim ioutf 1.0");
    ms = run_until(c_fault, 20);
    CHECK(ms > 0 && (qbc_faults() & F_OCP_OUT_F), "lim ioutf 1,0 A (Io 1,11 A) -> OCP_OUT_F dalam %.1f ms", ms);
}

static void t_otp(void)
{
    float ms;
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    CHECK(B.out.fan == 1, "kipas ON saat PWM jalan");
    B.temp_c = 70.0f; run_ms(100, NULL);
    CHECK(qbc_faults() == 0, "70 C belum trip");
    B.temp_c = 82.0f;
    ms = run_until(c_fault, 100);
    CHECK(ms > 0 && (qbc_faults() & F_OTP), "82 C -> OTP dalam %.1f ms (suhu dibaca tiap 10 ms)", ms);
    run_ms(10, NULL);
    CHECK(B.out.fan == 1, "kipas tetap ON setelah trip OTP (suhu > 35 C)");
    /* arm ditolak bila panas */
    run_ms(6000, NULL); cmd("clr"); run_ms(2, NULL);
    B.temp_c = 78.0f; run_ms(20, NULL);
    cmd("arm"); run_ms(5, NULL);
    CHECK(qbc_state() == ST_OFF && ev_seen("suhu_terlalu_tinggi"), "arm ditolak di 78 C (> tmax-5)");
    /* NTC lepas */
    B.temp_c = 25.0f; B.ntc_open = 1; run_ms(20, NULL);
    cmd("arm"); run_ms(5, NULL);
    CHECK(qbc_state() == ST_OFF && ev_seen("NTC_tidak_terbaca"), "arm ditolak saat NTC lepas");
    B.ntc_open = 0; run_ms(20, NULL);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal 2"); return; }
    B.ntc_open = 1;
    ms = run_until(c_fault, 100);
    CHECK(ms > 0 && (qbc_faults() & F_NTC), "NTC lepas saat jalan -> F_NTC dalam %.1f ms", ms);
}

static void t_dsat(void)
{
    const char *C[3] = { "hinf", "pi", "mpso" };
    int c;
    for (c = 0; c < 3; c++) {
        float ms;
        boot(1);
        if (!start_cl(C[c], 150)) { CHECK(0, "%s start gagal", C[c]); continue; }
        B.il1_max = 0;
        B.vsrc = 9.0f;                                   /* sumber jatuh: 150 V tidak tercapai */
        ms = run_until(c_fault, 1000);
        CHECK(ms > 0 && (qbc_faults() & (F_DSAT | F_OCP_IN_F | F_OCP_IN_S)),
              "%s: Vin jatuh ke 9 V -> trip dalam %.1f ms (faults 0x%x)", C[c], ms, qbc_faults());
        NOTE("%s: Vin 9 V -> fault %s setelah %.1f ms, iL1 puncak %.2f A",
             C[c], (qbc_faults() & F_DSAT) ? "DSAT" : (qbc_faults() & F_OCP_IN_F) ? "OCP_IN_F" : "lain", ms, B.il1_max);
    }
}

/* anti-windup. Pada beban penuh/setengah OCP_IN memotong lebih dulu sebelum duty menempel 0,85 (uji
 * dsat), dan step Vref +-20 V atau beban dilepas tidak membuat H-inf jenuh. Saturasi atas yang realistis:
 * beban ringan 1350 Ohm, sumber turun ke 6 V selama 100 ms (PSU drop / kabel), lalu kembali ke 18 V.
 * Ukur: lama duty jenuh SETELAH Vin pulih (windup), Vo puncak, iL1 puncak. dsat dinaikkan ke 200 ms
 * supaya perilaku pengendali terlihat (dengan dsat 50 ms -> uji dsat_chatter). */
static void aw_case(const char *ctrl, int aw, float *over, float *sat_after, float *il1pk)
{
    char nm[60];
    uint32_t i, hi = 0;
    boot(1);
    cmd(aw ? "aw on" : "aw off");
    cmd("lim dsat 200");
    B.R = 1350.0f;
    if (!start_cl(ctrl, 150)) { CHECK(0, "%s start gagal", ctrl); *over = *sat_after = *il1pk = -1; return; }
    snprintf(nm, sizeof nm, "AW_%s_aw%s_hw", ctrl, aw ? "on" : "off");
    trace_open(nm, 5);
    B.vsrc = 6.0f; run_ms(100, NULL);
    B.vsrc = 18.0f; B.vterm_max = 0; B.il1_max = 0;
    for (i = 0; i < 15000; i++) { run_ms(0.1f, NULL); if (qbc_state() == ST_RUN_CL && ctrl_last_sat() > 0) hi++; }
    trace_close();
    *over = B.vterm_max; *sat_after = hi * 0.1f; *il1pk = B.il1_max;
    NOTE("AW %s aw %s: setelah Vin pulih duty jenuh %.1f ms, Vo puncak %.2f V, iL1 puncak %.2f A, faults %s",
         ctrl, aw ? "on" : "off", *sat_after, *over, *il1pk, qbc_faults() ? qbc_fault_name(__builtin_ctz(qbc_faults())) : "-");
}
static void t_antiwindup(void)
{
    float o1, s1, i1, o0, s0, i0;
    aw_case("hinf", 1, &o1, &s1, &i1);
    aw_case("hinf", 0, &o0, &s0, &i0);
    NOTE("KESIMPULAN: anti-windup tidak bisa dibandingkan secara bermakna di plant ini: pada skenario yang membuat"
         " duty jenuh, OCP_IN_F trip tepat saat Vin pulih ke 18 V karena duty masih ~0,85 (iL1 puncak aw on %.1f A,"
         " aw off %.1f A; anti-windup conditional-integration tidak cukup cepat menurunkan duty); pada"
         " skenario lain (step Vref +-20 V, beban dilepas, S1-S3) duty tidak pernah jenuh. Pengaman utama = DSAT"
         " 50 ms (uji dsat_chat) dan OCP_IN.", i1, i0);
    CHECK(1, "dicatat saja (lihat info)");
    aw_case("pi", 1, &o1, &s1, &i1);
    aw_case("mpso", 1, &o0, &s0, &i0);
}

/* DSAT dengan penghitung bocor: skenario yang sama, dsat bawaan 50 ms */
static void t_dsat_chatter(void)
{
    float ms;
    boot(1);
    B.R = 1350.0f;
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    B.vsrc = 6.0f;
    ms = run_until(c_fault, 2000);
    CHECK(ms > 0 && (qbc_faults() & F_DSAT), "Vin 6 V @1350 Ohm (duty jenuh berselang-seling) -> F_DSAT setelah %.1f ms (faults 0x%x)", ms, qbc_faults());
}

static void t_plant_acc(void)
{
    /* akurasi integrator plant 10 sub-langkah vs 1000 sub-langkah, respons step D 0,60 -> 0,666 */
    qbc_plant_par_t p; qbc_plant_x_t xa, xb;
    int hw;
    for (hw = 0; hw < 2; hw++) {
        float dmax = 0, va = 0, vb = 0;
        uint32_t k; int j;
        if (hw) qbc_plant_par_hw(&p); else qbc_plant_par_model(&p);
        qbc_plant_reset(&xa); qbc_plant_reset(&xb);
        for (k = 0; k < 40000; k++) {
            float d = k < 20000 ? 0.60f : 0.666f;
            for (j = 0; j < 10; j++) va = qbc_plant_step(&p, &xa, d, 18, 135, 1e-5f);
            for (j = 0; j < 1000; j++) vb = qbc_plant_step(&p, &xb, d, 18, 135, 1e-7f);
            if (fabsf(va - vb) > dmax) dmax = fabsf(va - vb);
        }
        CHECK(dmax < 0.5f, "plant %s: 10 sub-langkah vs 1000: max |dVo| = %.3f V (step D 0,60 -> 0,666)", hw ? "hw" : "model", dmax);
        NOTE("plant %s D 0,666 @18 V/135 Ohm: Vo %.2f V, iL1 %.2f A", hw ? "hw" : "model", vb, xb.iL1);
    }
}

static void t_switch(void)
{
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    ev_clear();
    cmd("ctrl pi"); run_ms(2, NULL);
    CHECK(ev_seen("PWM_masih_jalan") && ctrl_selected() == CTRL_HINF, "ctrl pi ditolak saat RUN_CL");
    cmd("stop"); run_ms(2, NULL);
    CHECK(qbc_state() == ST_ARMED && !B.out.pwm_en, "stop -> ARMED, PWM mati, relai tetap");
    run_ms(300, NULL);
    cmd("ctrl pi"); run_ms(2, NULL);
    CHECK(ctrl_selected() == CTRL_PI, "ctrl pi diterima di ARMED");
    B.il1_max = 0;
    cmd("cl"); run_ms(3000, NULL);
    CHECK(qbc_state() == ST_RUN_CL && fabsf(B.vterm - 150) < 1.0f, "PI dari Vo sisa: kembali ke 150 V (%.2f)", B.vterm);
    NOTE("restart PI dari C2 bermuatan (Vo awal ~%s): iL1 puncak %.2f A", "turun dari 150 V selama 300 ms", B.il1_max);
    cmd("stop"); run_ms(2, NULL); cmd("ctrl mpso"); run_ms(2, NULL); cmd("cl"); run_ms(3000, NULL);
    CHECK(ctrl_selected() == CTRL_MPSO && fabsf(B.vterm - 150) < 1.0f, "pindah ke MPSO-PI: 150 V (%.2f)", B.vterm);
    cmd("stop"); run_ms(2, NULL); cmd("ctrl hinf"); run_ms(2, NULL); cmd("cl"); run_ms(3000, NULL);
    CHECK(ctrl_selected() == CTRL_HINF && fabsf(B.vterm - 150) < 1.0f && qbc_faults() == 0, "kembali ke H-inf: 150 V (%.2f), tanpa fault", B.vterm);
}

static void t_link(void)
{
    uint32_t t0; float ms;
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    B.link = 0; t0 = B.ticks;
    ms = run_until(c_pwmoff, 2000);
    CHECK(ms > 0 && fabsf(ms - LINK_LOSS_MS) < 1.0f, "USB putus -> PWM mati setelah %.1f ms (%u ms)", ms, LINK_LOSS_MS);
    CHECK(ev_seen("LINK_USB_PUTUS"), "event LINK_USB_PUTUS tercatat");
    run_until(c_off, 6000);
    CHECK(qbc_state() == ST_OFF, "lanjut DISARM -> OFF");
    cmd("arm"); run_ms(5, NULL);
    CHECK(qbc_state() == ST_OFF && ev_seen("link_USB_tidak_aktif"), "arm ditolak saat link mati");
    (void)t0;
    boot(1);
    cmd("link keep");
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    B.link = 0; run_ms(1500, NULL);
    CHECK(qbc_state() == ST_RUN_CL, "link keep: tetap jalan saat USB putus (pilihan eksplisit)");
}

static void t_k2_fail(void)
{
    float ms;
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    B.vc2_max = 0;
    trace_open("K2_lepas_hinf_hw", 1);
    B.k2_stuck_open = 1;                                 /* kontak K2 lepas: beban hilang + sensor Vo = 0 */
    ms = run_until(c_fault, 100);
    run_ms(20, NULL);
    trace_close();
    CHECK(ms > 0 && (qbc_faults() & (F_VO_PLAUS | F_DSAT)), "K2 lepas saat 150 V -> trip dalam %.1f ms (faults 0x%x)", ms, qbc_faults());
    NOTE("K2 lepas: Vo SEBENARNYA di C2 naik ke %.1f V sebelum PWM mati (sensor melihat 0 V)", B.vc2_max);
    CHECK(B.vc2_max < 250.0f, "Vo sebenarnya %.1f V < 250 V (rating C12 snubber) -- C11/C13 450 V", B.vc2_max);
    /* K2 tidak menutup saat arm */
    boot(1);
    B.k2_stuck_open = 1;
    cmd("arm");
    ms = run_until(c_fault, 1000);
    CHECK(ms > 0 && (qbc_faults() & F_ARM), "K2 tidak menutup saat arm -> F_ARM (%.0f ms), PWM tidak pernah jalan", ms);
}

static void t_ol(void)
{
    boot(1);
    if (!arm_ok()) { CHECK(0, "arm gagal"); return; }
    cmd("ol"); cmd("d 0.654");
    trace_open("T4_ol_0654_hw", 10);
    run_ms(4500, NULL);
    win_run(200);
    trace_close();
    CHECK(qbc_state() == ST_RUN_OL && qbc_faults() == 0, "OL D 0,654 @135 Ohm tanpa fault");
    CHECK(fabsf((float)(W.d_sum / W.n) - 0.654f) < 1e-3f, "duty OL = %.4f", W.d_sum / W.n);
    NOTE("T4 OL D 0,654 @135 Ohm (set hw): Vo %.2f V, iL1 %.2f A", W.vo_sum / W.n, W.il1_sum / W.n);
    cmd("d 0.8"); run_ms(2000, NULL);
    CHECK(fabsf(B.out.duty - OL_DMAX_DEFAULT) < 1e-4f, "d 0.8 di OL dijepit ke dmax %.2f (duty %.4f)", OL_DMAX_DEFAULT, B.out.duty);
    cmd("d 0"); run_ms(4000, NULL);
    CHECK(B.out.duty == 0.0f, "d 0 -> ramp turun ke 0");
    cmd("stop"); run_ms(2, NULL);
    CHECK(qbc_state() == ST_ARMED, "stop -> ARMED");
}

static void t_ol_noload(void)
{
    float ms;
    boot(1);
    if (!arm_ok()) { CHECK(0, "arm gagal"); return; }
    B.R = -1.0f;                                         /* beban lupa dipasang (K2 tertutup) */
    B.vc2_max = 0;
    cmd("ol"); cmd("d 0.5");
    ms = run_until(c_fault, 6000);
    CHECK(ms > 0 && (qbc_faults() & (F_NOLOAD | F_OVP)), "OL tanpa beban -> trip (%s) setelah %.0f ms",
          (qbc_faults() & F_NOLOAD) ? "NOLOAD" : (qbc_faults() & F_OVP) ? "OVP" : "?", ms);
    NOTE("OL tanpa beban: Vo puncak %.1f V sebelum trip", B.vc2_max);
}

static void t_pwmtest(void)
{
    float ms;
    boot(1);
    cmd("pwmtest"); run_ms(5, NULL);
    CHECK(qbc_state() == ST_OFF && ev_seen("Vin_ada_saat_pwmtest"), "pwmtest ditolak saat PSU 18 V terpasang");
    B.vsrc = 0.0f; run_ms(2, NULL);
    cmd("pwmtest"); cmd("d 0.5"); run_ms(3000, NULL);
    CHECK(qbc_state() == ST_PWMTEST && B.out.pwm_en && fabsf(B.out.duty - 0.5f) < 1e-4f && !B.out.k1 && !B.out.k2,
          "pwmtest (Vin 0): PWM 0,5 dengan K1/K2 terbuka");
    CHECK(B.vc2_max < 0.01f, "tidak ada daya ke power stage selama pwmtest");
    cmd("d 0.85"); run_ms(5000, NULL);
    CHECK(fabsf(B.out.duty - 0.85f) < 1e-4f, "pwmtest boleh sampai 0,85 (duty %.4f)", B.out.duty);
    B.vsrc = 18.0f;                                       /* PSU dinyalakan di tengah pwmtest */
    ms = run_until(c_fault, 10);
    CHECK(ms > 0 && (qbc_faults() & F_RELAY), "PSU dinyalakan saat pwmtest -> F_RELAY dalam %.1f ms", ms);
    /* force + K1 lengket */
    boot(1);
    B.k1_welded = 1; B.vc2_max = 0;
    run_ms(50, NULL);
    cmd("pwmtest force"); cmd("d 0.5");
    ms = run_until(c_fault, 3000);
    NOTE("BATASAN: pwmtest force + K1 lengket: %s setelah %.1f ms, Vo C2 naik ke %.1f V tanpa terlihat sensor."
         " Karena itu default pwmtest menolak Vin > 2 V; force hanya bila K1 sudah dicek dengan multimeter.",
         (qbc_faults() & F_RELAY) ? "F_RELAY" : qbc_faults() ? "fault lain" : "TIDAK TERDETEKSI", ms, B.vc2_max);
    /* force tanpa kelainan */
    boot(1);
    cmd("pwmtest force"); cmd("d 0.654"); run_ms(4000, NULL);
    CHECK(qbc_state() == ST_PWMTEST && qbc_faults() == 0, "pwmtest force normal (Vin 18 V, relai sehat): jalan tanpa trip palsu");
}

static void t_sim(void)
{
    boot(1);
    cmd("sim par hw"); cmd("sim on"); run_ms(2, NULL);
    cmd("arm"); run_until(c_armed, 1000);
    cmd("v 150"); cmd("cl"); run_ms(1500, NULL);
    {
        char st[400];
        qbc_command("st", st, sizeof st);
        fprintf(flog, "    %s", st);
        CHECK(!B.out.k1 && !B.out.k2, "SIM: pin relai fisik tetap LOW");
        CHECK(qbc_state() == ST_RUN_CL && strstr(st, "vo=1") != NULL, "SIM: closed-loop di model on-chip mencapai ~150 V");
        CHECK(B.vc2_max < 0.01f, "SIM: power stage asli tidak menerima daya");
    }
    B.k1_welded = 1;                                     /* ADC asli tiba-tiba melihat daya */
    {
        float ms = run_until(c_fault, 2000);
        NOTE("SIM + K1 lengket: %s setelah %.1f ms", (qbc_faults() & F_SIM_REAL) ? "F_SIM_REAL" : qbc_faults() ? "fault lain" : "tidak trip", ms);
        CHECK(ms > 0, "SIM berhenti bila ADC asli melihat daya (faults 0x%x)", qbc_faults());
    }
}

static void t_start_inrush(void)
{
    boot(1);
    cmd("vr 6000");
    if (!arm_ok()) { CHECK(0, "arm gagal"); return; }
    B.il1_max = 0;
    cmd("v 150"); cmd("cl");
    run_ms(1500, NULL);
    NOTE("vr 6000 V/s (= MIL): iL1 puncak saat start %.2f A, faults 0x%x", B.il1_max, qbc_faults());
    CHECK(1, "dicatat saja");
}

static void t_arming_brownout(void)
{
    boot(1);
    cmd("arm"); run_ms(50, NULL);
    B.vsrc = 3.0f;
    run_ms(400, NULL);
    CHECK(qbc_state() == ST_FAULT && (qbc_faults() & F_ARM), "sumber jatuh saat ARMING -> F_ARM (state %s)", qbc_state_name(qbc_state()));
    CHECK(!B.out.k2, "K2 tidak pernah ditutup");
}

static void t_disarm_hv(void)
{
    uint32_t t0;
    boot(1);
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    t0 = B.ticks; t_k1open = t_k2open = 0;
    trace_open("disarm_150V_hw", 10);
    cmd("disarm");
    run_ms(6000, hook_open);
    trace_close();
    CHECK(qbc_state() == ST_OFF, "disarm dari 150 V -> OFF");
    NOTE("disarm dari 150 V: K1 buka %.1f ms, K2 buka %.0f ms, Vo saat K2 dibuka %.1f V",
         (t_k1open - t0) * 0.1f, (t_k2open - t0) * 0.1f, vo_k2open);
    CHECK(vo_k2open < K2_OPEN_VO_V + 1, "K2 dibuka saat Vo < 30 V");
}

static void t_queue_raw_cap(void)
{
    int i, okq = 0, err = 0, lines = 0, end = 0, hdr = 0;
    char b[400];
    boot(1);
    for (i = 0; i < 30; i++) { qbc_command("trig", b, sizeof b); if (strstr(b, "OK")) okq++; else if (strstr(b, "penuh")) err++; }
    CHECK(err > 0 && okq == 15, "antrean 16 slot: %d diterima, %d ditolak 'antrean penuh', tidak crash", okq, err);
    run_ms(5, NULL);
    cmd("raw 1000"); run_ms(200, NULL);
    {
        int n = qbc_poll_output(b, sizeof b);
        fprintf(flog, "    %s", n > 0 ? b : "(kosong)\n");
        CHECK(n > 0 && strncmp(b, "R,n=1000", 8) == 0, "raw 1000 -> baris R,n=1000,...");
    }
    if (!start_cl("hinf", 150)) { CHECK(0, "start gagal"); return; }
    cmd("cap 20 1"); run_ms(100, NULL);
    cmd("s 5"); run_ms(500, NULL);
    CHECK(ev_seen("CAPTURE_SELESAI"), "capture selesai setelah step");
    cmd("capdump");
    {
        float vmax = 0;
        while ((i = qbc_poll_output(b, sizeof b)) > 0) {
            lines++;
            if (strstr(b, "C,HDR")) hdr = 1;
            if (strstr(b, "C,END")) { end = 1; break; }
            if (lines > 3 && lines < 5000) { float t, v; int idx; if (sscanf(b, "C,%d,%f,%f", &idx, &t, &v) == 3 && v > vmax) vmax = v; }
        }
        CHECK(hdr && end && lines == (int)CAP_N + 1, "capdump: %d panggilan (header+sampel 0, %d sampel, END)", lines, CAP_N);
        NOTE("capture S1: Vo maks di buffer %.2f V", vmax);
    }
    /* telemetri */
    {
        uint32_t now = 1000; int got = 0;
        cmd("tel 100");
        for (i = 0; i < 3000 && !got; i++) { int n; tick1(); now += 0; if ((n = qbc_poll_telemetry(now + i / 10, b, sizeof b)) > 0) got = 1; }
        fprintf(flog, "    %s", got ? b : "(tidak ada telemetri)\n");
        {
            int commas = 0; char *p;
            for (p = b; *p; p++) if (*p == ',') commas++;
            CHECK(got && strncmp(b, "T,", 2) == 0 && commas == 19, "telemetri T,... 20 kolom (%d koma)", commas);
        }
    }
}

static void t_cal(void)
{
    char b[300];
    boot(1);
    cmd("cal vo 0.05 0");
    CHECK(strstr(lastrep, "OK") != NULL, "cal vo 0,05 (dekat nominal 0,0484) diterima");
    cmd("cal vo 0.0217 0");
    CHECK(strstr(lastrep, "ERR") != NULL, "cal vo dengan skala v1 (88,96/4095 = 0,0217) ditolak (>50 %% dari skematik)");
    if (!arm_ok()) { CHECK(0, "arm gagal"); return; }
    qbc_command("cal vo 0.05 0", b, sizeof b);
    CHECK(strstr(b, "ERR") != NULL, "cal ditolak saat tidak OFF");
}

/* ======================================================================================
 *  Main
 * ====================================================================================== */
typedef struct { const char *name; void (*fn)(void); const char *desc; } test_t;
static const test_t TESTS[] = {
    { "boot_safe",   t_boot_safe,      "boot aman: PWM/relai mati, tanpa fault palsu" },
    { "refuse",      t_refuse,         "perintah berbahaya ditolak di status yang salah, batas keras" },
    { "arm_seq",     t_arm_seq,        "urutan relai K1 -> 300 ms -> K2, disarm" },
    { "ramp150",     t_ramp150,        "ramp ke 150 V @135 Ohm: H-inf, PI-FR, MPSO-PI (set hw & model)" },
    { "plant_acc",   t_plant_acc,      "akurasi integrator plant harness/SIM" },
    { "load_step",   t_load_step,      "S2 (panduan 4.9): beban 135 -> 270 -> 135 Ohm" },
    { "s2_mil",      t_s2_mil,         "S2 versi MIL-2 (Gambar 4.10): 135 -> 101 Ohm (info)" },
    { "vref_step",   t_vref_step,      "S1: Vref 150 -> 155 V + pin TRIG" },
    { "vin_step",    t_vin_step,       "S3: Vin 18 -> 19,8 V dan 18 -> 16,2 V tanpa trip palsu" },
    { "vin_slew",    t_vin_slew,       "S3 arah kembali dengan slew PSU 0/1/10 ms (info)" },
    { "ovp",         t_ovp,            "OVP (ambang diturunkan 152 V) + urutan buka relai + latch" },
    { "estop_clr",   t_ovp_clr_early,  "'!' darurat, clr ditolak sebelum relai terbuka" },
    { "ocp_out",     t_ocp_out,        "hubung singkat keluaran 5 Ohm" },
    { "ocp_in",      t_ocp_in,         "OCP masukan cepat/lambat dan keluaran (ambang diturunkan)" },
    { "otp",         t_otp,            "overtemp, NTC lepas, kipas" },
    { "dsat",        t_dsat,           "duty jenuh (Vin jatuh 9 V) -> trip" },
    { "antiwindup",  t_antiwindup,     "saturasi duty atas (Vin 6 V 100 ms) dan anti-windup" },
    { "dsat_chat",   t_dsat_chatter,   "DSAT tetap trip walau duty jenuh berselang-seling" },
    { "switch",      t_switch,         "perpindahan pengendali hinf -> pi -> mpso -> hinf" },
    { "link",        t_link,           "USB putus" },
    { "k2_fail",     t_k2_fail,        "K2 lepas saat jalan / tidak menutup saat arm" },
    { "ol",          t_ol,             "open-loop T4 (D 0,654), jepit dmax" },
    { "ol_noload",   t_ol_noload,      "open-loop tanpa beban" },
    { "pwmtest",     t_pwmtest,        "pwmtest T1/T2, K1 lengket" },
    { "sim",         t_sim,            "mode SIM on-chip" },
    { "inrush",      t_start_inrush,   "arus start dengan vr 6000 V/s (info)" },
    { "arm_brown",   t_arming_brownout,"sumber jatuh saat ARMING" },
    { "disarm_hv",   t_disarm_hv,      "disarm dari 150 V" },
    { "io",          t_queue_raw_cap,  "antrean perintah, raw, capture, telemetri" },
    { "cal",         t_cal,            "perintah kalibrasi" },
};
#define NT (int)(sizeof(TESTS) / sizeof(TESTS[0]))

/* Satu uji = satu proses (state statis firmware bersih seperti setelah reset). Induk memanggil dirinya
 * sendiri:  harness --child <nama>  lewat popen(), jadi jalan di Linux, WSL, MSYS2 maupun MinGW biasa. */
static int run_child(const char *name)
{
    int i;
    for (i = 0; i < NT; i++) if (strcmp(TESTS[i].name, name) == 0) break;
    if (i == NT) { printf("RES 1 1\n    GAGAL: uji tidak dikenal\n"); return 1; }
    flog = fopen("hasil/hasil_uji.txt", "a");
    if (!flog) flog = stderr;
    fprintf(flog, "=== %s: %s ===\n", TESTS[i].name, TESTS[i].desc);
    srand(12345);
    TESTS[i].fn();
    if (inv_fail) {
        n_chk++; n_fail++; fprintf(flog, "  [GAGAL] INVARIAN: %s\n", inv_msg);
        strncat(summary, "    GAGAL INVARIAN: ", sizeof(summary) - strlen(summary) - 1);
        strncat(summary, inv_msg, sizeof(summary) - strlen(summary) - 1);
        strncat(summary, "\n", sizeof(summary) - strlen(summary) - 1);
    } else fprintf(flog, "  [LOLOS] invarian keselamatan dijaga di setiap tick\n");
    fprintf(flog, "\n");
    if (flog != stderr) fclose(flog);
    printf("RES %d %d\n%s", n_chk + 1, n_fail, summary);
    return 0;
}

int main(int argc, char **argv)
{
    int i, j, tot_c = 0, tot_f = 0, tests_failed = 0, ran = 0;
    FILE *fs;
    if (argc == 3 && strcmp(argv[1], "--child") == 0) return run_child(argv[2]);
    fs = fopen("hasil/hasil_uji.txt", "w");
    if (!fs) { printf("buat folder 'hasil' dulu (make membuatnya)\n"); return 2; }
    fprintf(fs, "Harness uji PC firmware QBC v2 (%s)\n", QBC_FW_VERSION);
    fprintf(fs, "Plant: model rata-rata qbc_sim.c, Vin 18 V, R 135 Ohm, set parasitik 'hw' kecuali disebut 'model'\n\n");
    fclose(fs);
    for (i = 0; i < NT; i++) {
        int sel = (argc < 2), res_c = 1, res_f = 1, got = 0;
        char cmdline[600], line[600], sm[6000] = "";
        FILE *p;
        for (j = 1; j < argc; j++) if (strcmp(argv[j], TESTS[i].name) == 0) sel = 1;
        if (!sel) continue;
        ran++;
        snprintf(cmdline, sizeof cmdline, "\"%s\" --child %s", argv[0], TESTS[i].name);
        fflush(stdout);
        p = popen(cmdline, "r");
        if (p) {
            while (fgets(line, sizeof line, p)) {
                if (!got && sscanf(line, "RES %d %d", &res_c, &res_f) == 2) got = 1;
                else if (got && strlen(sm) + strlen(line) < sizeof sm - 1) strcat(sm, line);
            }
            if (pclose(p) != 0 || !got) { res_f++; strcat(sm, "    GAGAL: proses uji crash / tidak melapor\n"); }
        } else { res_f++; strcat(sm, "    GAGAL: popen gagal\n"); }
        printf("%-11s %-6s %2d cek  %s\n", TESTS[i].name, res_f ? "GAGAL" : "LOLOS", res_c, TESTS[i].desc);
        if (sm[0]) fputs(sm, stdout);
        tot_c += res_c; tot_f += res_f; if (res_f) tests_failed++;
    }
    printf("\nTOTAL: %d uji, %d cek, %d gagal (%d uji gagal)\n", ran, tot_c, tot_f, tests_failed);
    fs = fopen("hasil/hasil_uji.txt", "a");
    if (fs) { fprintf(fs, "TOTAL: %d uji, %d cek, %d gagal\n", ran, tot_c, tot_f); fclose(fs); }
    return tot_f ? 1 : 0;
}
