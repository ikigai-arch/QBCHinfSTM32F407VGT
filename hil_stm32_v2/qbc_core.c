/* qbc_core.c -- lihat qbc_core.h. C99 portabel: dipakai di STM32 (ISR ADC) dan di harness PC. */
#include "qbc_core.h"
#include "qbc_config.h"
#include "qbc_sim.h"
#include "ctrl_mux.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ======================================================================================
 *  Konfigurasi runtime
 * ====================================================================================== */
enum { CH_VO = 0, CH_VIN = 1, CH_IIN = 2, CH_IOUT = 3, CH_NCAL = 4 };
static const char *const CH_NAME[CH_NCAL] = { "vo", "vin", "iin", "iout" };
typedef struct { float a, b; } cal_t;
static cal_t cal[CH_NCAL];

typedef struct {
    float    ovp, iin_f, iin_s, iout_f, iout_s, vin_max, tmax;
    uint32_t dsat_ms;
} lim_t;
static lim_t lim;

static volatile int   ntc_check   = 1;
static volatile int   noload_chk  = 1;
static volatile int   link_stop   = 1;
static volatile int   fan_mode    = 0;    /* 0 auto, 1 on, 2 off */
static volatile float drate       = OL_DRATE_DEFAULT;
static volatile float ol_dmax     = OL_DMAX_DEFAULT;
static volatile float vrate       = VREF_RATE_DEFAULT;
static volatile uint32_t tel_ms   = TEL_MS_DEFAULT;

/* ======================================================================================
 *  Status (ditulis ISR)
 * ====================================================================================== */
static volatile int      st = ST_OFF;
static volatile uint32_t faults = 0, fault_first = 0;
static volatile uint32_t tick = 0;
static uint32_t st_ticks = 0;            /* tick sejak masuk status/fase sekarang */
static int      phase = 0;

static float vo, vin, iin, iout, temp = 25.0f;
static int   ntc_bad = 0;
static float iin_avg = 0.0f, iout_avg = 0.0f;
static uint32_t c_ovp, c_iinf, c_ioutf, c_plaus, c_noload, c_dsat, c_adcsat, c_relay, c_simreal, c_vodrop;
static float    vo_lp = 0.0f;
static int      pwmtest_force = 0;

static float duty_out = 0.0f;
static int   pwm_en = 0, k1 = 0, k2 = 0, fan = 0;
static uint32_t trig_left = 0;
static int   trig_level = 0;
static float duty_cmd = 0.0f, duty_now = 0.0f;
static float vref_target = 0.0f, vref_now = 0.0f;
static int   sample_midoff = 0;
static float t_sample_us = 50.0f;
static float vo_at_k2open = 0.0f;

static volatile int link_ok = 1;
static uint32_t link_lost_ticks = 0;

/* SIM */
static int sim_on = 0;
static qbc_plant_par_t simp;
static qbc_plant_x_t   simx;
static volatile float  sim_R = 135.0f, sim_vin = 18.0f;
static int             sim_hw = 0;

/* statistik HW */
static volatile uint32_t hw_isr_cyc = 0, hw_isr_cyc_max = 0, hw_ovr = 0, hw_sysclk = 0;
static volatile int      hw_hse = -1;
static volatile uint32_t ext_fault = 0;
static uint32_t sat_ticks_total = 0;

/* ======================================================================================
 *  Antrean perintah (loop -> ISR) dan event (ISR -> loop)
 * ====================================================================================== */
enum {
    OP_ARM = 1, OP_DISARM, OP_STOP, OP_KILL, OP_CLR, OP_PWMTEST, OP_OL, OP_CL, OP_DUTY, OP_VREF,
    OP_STEP, OP_TRIG, OP_CTRL, OP_SIM, OP_CAPGO, OP_AW
};
typedef struct { uint8_t op; float a; } qcmd_t;
#define CQ_N 16u
static qcmd_t cq[CQ_N];
static volatile uint32_t cq_head = 0, cq_tail = 0;

static int cq_push(uint8_t op, float a)
{
    uint32_t h = cq_head, nx = (h + 1u) % CQ_N;
    if (nx == cq_tail) return 0;
    cq[h].op = op; cq[h].a = a;
    cq_head = nx;
    return 1;
}

enum { EV_STATE = 1, EV_FAULT, EV_REFUSE, EV_INFO };
enum {
    R_STATE = 1, R_VIN, R_VOEXT, R_NTC, R_TEMP, R_FAULTACT, R_VREF0, R_RELAYS, R_SIMREAL,
    R_PWMON, R_LINK, R_PWMTEST_VIN
};
enum { I_TRIG = 1, I_CTRL, I_CAPDONE, I_K2OPEN_HV, I_LINKLOST, I_SIM, I_AW, I_STEP, I_ARMSTEP };
typedef struct { uint32_t tick; uint8_t type; uint8_t code; float val; } ev_t;
#define EV_N 32u
static ev_t evq[EV_N];
static volatile uint32_t ev_head = 0, ev_tail = 0;
static volatile uint32_t ev_drop = 0;

static void ev_push(uint8_t type, uint8_t code, float val)
{
    uint32_t h = ev_head, nx = (h + 1u) % EV_N;
    if (nx == ev_tail) { ev_drop++; return; }
    evq[h].tick = tick; evq[h].type = type; evq[h].code = code; evq[h].val = val;
    ev_head = nx;
}

/* ======================================================================================
 *  Telemetri, statistik raw, capture
 * ====================================================================================== */
typedef struct { uint32_t n; float vo, vin, iin, iout, duty, vomax, vomin; } acc_t;
static acc_t acc, snap;
static volatile int snap_req = 0, snap_ready = 0;

static volatile uint32_t raw_target = 0;
static uint32_t raw_n = 0;
static uint64_t rs[5], rss[5];
static volatile int raw_ready = 0;

typedef struct { uint16_t vo, vin, iout, duty; } cap_t;
static cap_t capbuf[CAP_N];
enum { CAP_IDLE = 0, CAP_ARMED, CAP_POST, CAP_DONE };
static volatile int cap_state = CAP_IDLE;
static uint32_t cap_wr = 0, cap_filled = 0, cap_post_left = 0, cap_pre = 0;
static uint32_t cap_dec = 1, cap_dec_cnt = 0, cap_trig_tick = 0;
static int      cap_trig_src = 0;
static uint32_t dump_idx = 0;
static volatile int dump_active = 0;

/* ======================================================================================
 *  Utilitas
 * ====================================================================================== */
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static float ntc_temp(uint16_t code)
{
    float v, r;
    if (code < NTC_OPEN_CODE || code > NTC_SHORT_CODE) return NAN;
    /* V = 3,3 * R11/(Rntc+R11); kode/4095 = R11/(Rntc+R11) (rasio, sumber 3,3 V dianggap sama) */
    v = (float)code / 4095.0f;
    r = NTC_RPULL_OHM * (1.0f / v - 1.0f);
    return 1.0f / (1.0f / NTC_T0_K + logf(r / NTC_R0_OHM) / NTC_BETA) - 273.15f;
}

static void set_state(int s)
{
    if (st != s) {
        st = s;
        ev_push(EV_STATE, (uint8_t)s, 0.0f);
    }
    st_ticks = 0;
    phase = 0;
}

static void cap_trigger(int src)
{
    if (cap_state == CAP_ARMED) {
        cap_state = CAP_POST;
        cap_post_left = CAP_N - cap_pre;
        cap_trig_tick = tick;
        cap_trig_src = src;
    }
}

static void trip(uint32_t f, float val)
{
    int bit = 0;
    faults |= f;
    while (bit < 31 && !((f >> bit) & 1u)) bit++;
    ev_push(EV_FAULT, (uint8_t)bit, val);
    pwm_en = 0;
    duty_out = 0.0f;
    if (st != ST_FAULT) {
        fault_first = f;
        cap_trigger(3);
        set_state(ST_FAULT);
    }
}

static void refuse(uint8_t why) { ev_push(EV_REFUSE, why, 0.0f); }

static int pwm_state(int s) { return s == ST_RUN_OL || s == ST_RUN_CL || s == ST_PWMTEST; }

/* ======================================================================================
 *  Inisialisasi
 * ====================================================================================== */
static void lim_defaults(void)
{
    lim.ovp     = OVP_V_DEFAULT;
    lim.iin_f   = IIN_FAST_DEFAULT;
    lim.iin_s   = IIN_SLOW_DEFAULT;
    lim.iout_f  = IOUT_FAST_DEFAULT;
    lim.iout_s  = IOUT_SLOW_DEFAULT;
    lim.vin_max = VIN_MAX_DEFAULT;
    lim.tmax    = TEMP_TRIP_DEFAULT;
    lim.dsat_ms = DSAT_MS_DEFAULT;
}

void qbc_init(void)
{
    cal[CH_VO].a = CAL_VO_A;     cal[CH_VO].b = CAL_VO_B;
    cal[CH_VIN].a = CAL_VIN_A;   cal[CH_VIN].b = CAL_VIN_B;
    cal[CH_IIN].a = CAL_IIN_A;   cal[CH_IIN].b = CAL_IIN_B;
    cal[CH_IOUT].a = CAL_IOUT_A; cal[CH_IOUT].b = CAL_IOUT_B;
    lim_defaults();
    st = ST_OFF; faults = 0; fault_first = 0; tick = 0; st_ticks = 0; phase = 0;
    pwm_en = 0; k1 = 0; k2 = 0; fan = 0; duty_out = 0.0f; duty_cmd = 0.0f; duty_now = 0.0f;
    vref_target = 0.0f; vref_now = 0.0f; trig_left = 0; trig_level = 0;
    iin_avg = 0.0f; iout_avg = 0.0f;
    c_ovp = c_iinf = c_ioutf = c_plaus = c_noload = c_dsat = c_adcsat = c_relay = c_simreal = c_vodrop = 0;
    cq_head = cq_tail = 0; ev_head = ev_tail = 0;
    memset(&acc, 0, sizeof(acc));
    sim_on = 0; qbc_plant_par_model(&simp); qbc_plant_reset(&simx);
    ctrl_set_aw(1);
    ctrl_select(CTRL_HINF);
    ev_push(EV_STATE, ST_OFF, 0.0f);
}

/* ======================================================================================
 *  Eksekusi perintah antrean (konteks ISR)
 * ====================================================================================== */
static void do_cmd(const qcmd_t *c)
{
    switch (c->op) {
    case OP_KILL:
        trip(F_USER, 0.0f);
        break;
    case OP_STOP:
        if (st == ST_RUN_OL || st == ST_RUN_CL) { pwm_en = 0; set_state(ST_ARMED); }
        else if (st == ST_PWMTEST) { pwm_en = 0; set_state(ST_OFF); }
        else if (st == ST_ARMING) { set_state(ST_DISARM); }
        break;
    case OP_DISARM:
        if (st == ST_OFF || st == ST_FAULT || st == ST_DISARM) break;
        pwm_en = 0;
        if (st == ST_PWMTEST) { set_state(ST_OFF); break; }
        set_state(ST_DISARM);
        break;
    case OP_CLR:
        if (st != ST_FAULT) { faults = 0; break; }
        if (phase < 2) { refuse(R_RELAYS); break; }      /* relai belum selesai dibuka */
        faults = 0; fault_first = 0; ext_fault = 0;
        c_ovp = c_iinf = c_ioutf = c_plaus = c_noload = c_dsat = c_adcsat = c_relay = c_simreal = c_vodrop = 0;
        iin_avg = 0.0f; iout_avg = 0.0f;
        set_state(ST_OFF);
        break;
    case OP_ARM:
        if (st != ST_OFF) { refuse(R_STATE); break; }
        if (!link_ok && link_stop) { refuse(R_LINK); break; }
        if (vin < VIN_MIN_ARM || vin > lim.vin_max) { refuse(R_VIN); break; }
        if (vo > ARM_VO_EXT_MAX) { refuse(R_VOEXT); break; }
        if (ntc_check && ntc_bad) { refuse(R_NTC); break; }
        if (!ntc_bad && temp > lim.tmax - 5.0f) { refuse(R_TEMP); break; }
        set_state(ST_ARMING);
        k1 = 1;                                   /* K1 dulu: C11/C13 terisi ~Vin sebelum beban */
        ev_push(EV_INFO, I_ARMSTEP, 1.0f);
        break;
    case OP_PWMTEST:
        if (st != ST_OFF) { refuse(R_STATE); break; }
        if (vo > SIM_REAL_VO_MAX) { refuse(R_VOEXT); break; }
        pwmtest_force = (c->a > 0.5f);
        if (!pwmtest_force && vin > PWMTEST_VIN_MAX) { refuse(R_PWMTEST_VIN); break; }
        duty_cmd = 0.0f; duty_now = 0.0f;
        set_state(ST_PWMTEST);
        pwm_en = 1;
        break;
    case OP_OL:
        if (st != ST_ARMED) { refuse(R_STATE); break; }
        duty_cmd = 0.0f; duty_now = 0.0f;
        c_noload = 0;
        set_state(ST_RUN_OL);
        pwm_en = 1;
        break;
    case OP_CL:
        if (st != ST_ARMED) { refuse(R_STATE); break; }
        if (vref_target <= 0.0f) { refuse(R_VREF0); break; }
        ctrl_reset();
        vref_now = vo;                            /* ramp mulai dari Vo terukur (~Vin) */
        c_dsat = 0;
        set_state(ST_RUN_CL);
        pwm_en = 1;
        break;
    case OP_DUTY: {
        float dmax = (st == ST_PWMTEST) ? PWMTEST_DMAX : ol_dmax;
        duty_cmd = clampf(c->a, 0.0f, dmax);
        break;
    }
    case OP_VREF:
        vref_target = clampf(c->a, 0.0f, VREF_MAX_HARD);
        break;
    case OP_STEP:
        if (st != ST_RUN_CL) { refuse(R_STATE); break; }
        vref_target = clampf(vref_target + c->a, 0.0f, VREF_MAX_HARD);
        vref_now    = clampf(vref_now + c->a, 0.0f, VREF_MAX_HARD);
        trig_left = 10; cap_trigger(1);
        ev_push(EV_INFO, I_STEP, vref_now);
        break;
    case OP_TRIG:
        trig_left = 10; cap_trigger(2);
        ev_push(EV_INFO, I_TRIG, 0.0f);
        break;
    case OP_CAPGO:
        cap_trigger(4);
        break;
    case OP_CTRL:
        if (pwm_en || pwm_state(st)) { refuse(R_PWMON); break; }
        ctrl_select((int)c->a);
        ev_push(EV_INFO, I_CTRL, c->a);
        break;
    case OP_AW:
        ctrl_set_aw(c->a > 0.5f);
        ev_push(EV_INFO, I_AW, c->a);
        break;
    case OP_SIM:
        if (st != ST_OFF) { refuse(R_STATE); break; }
        sim_on = (c->a > 0.5f);
        qbc_plant_reset(&simx);
        c_simreal = 0;
        ev_push(EV_INFO, I_SIM, (float)sim_on);
        break;
    default:
        break;
    }
}

/* ======================================================================================
 *  ISR: satu langkah Ts
 * ====================================================================================== */
void qbc_isr(const qbc_raw_t *raw, qbc_out_t *out)
{
    float vo_real, vin_real, iin_real, iout_real;
    int powered;

    tick++;
    st_ticks++;

    /* ---- 1) Pengukuran ---- */
    vo_real   = cal[CH_VO].a   * (float)raw->vo   + cal[CH_VO].b;
    vin_real  = cal[CH_VIN].a  * (float)raw->vin  + cal[CH_VIN].b;
    iin_real  = cal[CH_IIN].a  * (float)raw->iin  + cal[CH_IIN].b;
    iout_real = cal[CH_IOUT].a * (float)raw->iout + cal[CH_IOUT].b;
    if ((tick % 100u) == 1u) {                    /* suhu lambat: cukup tiap 10 ms (logf mahal) */
        float t = ntc_temp(raw->temp);
        ntc_bad = (t != t);
        temp = ntc_bad ? 25.0f : t;
    }

    if (sim_on) {
        /* plant rata-rata di chip; duty yang berlaku periode ini = duty_out langkah lalu */
        int k;
        float vterm = 0.0f;
        float vin_eff = k1 ? sim_vin : 0.0f;
        float R = k2 ? sim_R : -1.0f;
        float d = pwm_en ? duty_out : 0.0f;
        for (k = 0; k < 10; k++) vterm = qbc_plant_step(&simp, &simx, d, vin_eff, R, QBC_TS / 10.0f);
        vo   = k2 ? vterm : 0.0f;                 /* sensor V_OUT ada setelah K2 */
        vin  = sim_vin;                           /* sensor V_IN ada sebelum K1 */
        iin  = simx.iL1;
        iout = (k2 && sim_R > 0.0f) ? vterm / sim_R : 0.0f;
        /* ADC asli tidak boleh melihat daya selama SIM */
        if (vo_real > SIM_REAL_VO_MAX || iin_real > 1.0f || iout_real > 0.5f) {
            if (++c_simreal >= 5) trip(F_SIM_REAL, vo_real);
        } else c_simreal = 0;
    } else {
        vo = vo_real; vin = vin_real; iin = iin_real; iout = iout_real;
    }
    iin_avg  += (iin  - iin_avg)  * (QBC_TS / ISLOW_TAU_S);
    iout_avg += (iout - iout_avg) * (QBC_TS / ISLOW_TAU_S);

    /* ---- 2) Statistik raw (T3), telemetri ---- */
    if (raw_target && !raw_ready) {
        const uint16_t v[5] = { raw->vo, raw->vin, raw->iin, raw->iout, raw->temp };
        int i;
        for (i = 0; i < 5; i++) { rs[i] += v[i]; rss[i] += (uint64_t)v[i] * v[i]; }
        if (++raw_n >= raw_target) raw_ready = 1;
    }
    acc.n++;
    acc.vo += vo; acc.vin += vin; acc.iin += iin; acc.iout += iout; acc.duty += pwm_en ? duty_out : 0.0f;
    if (acc.n == 1 || vo > acc.vomax) acc.vomax = vo;
    if (acc.n == 1 || vo < acc.vomin) acc.vomin = vo;
    if (snap_req && !snap_ready) { snap = acc; memset(&acc, 0, sizeof(acc)); snap_req = 0; snap_ready = 1; }

    /* ---- 3) Perintah ---- */
    while (cq_tail != cq_head) {
        qcmd_t c = cq[cq_tail];
        cq_tail = (cq_tail + 1u) % CQ_N;
        do_cmd(&c);
    }
    if (ext_fault) { uint32_t f = ext_fault; ext_fault = 0; trip(f, 0.0f); }

    /* ---- 4) Proteksi ---- */
    powered = k1 || k2 || pwm_en;
    if (powered) {
        /* OVP */
        if (vo > lim.ovp) { if (++c_ovp >= OVP_N) trip(F_OVP, vo); } else c_ovp = 0;
        /* ADC jenuh (V_OUT >= 198 V atau I_IN >= 22 A tidak terukur lagi) */
        if (!sim_on && (raw->vo >= 4090u || raw->iin >= 4090u)) { if (++c_adcsat >= 2) trip(F_ADC_SAT, (float)raw->vo); }
        else c_adcsat = 0;
        /* OCP masukan & keluaran */
        if (iin > lim.iin_f)  { if (++c_iinf  >= IIN_FAST_N)  trip(F_OCP_IN_F,  iin); }  else c_iinf = 0;
        if (iout > lim.iout_f){ if (++c_ioutf >= IOUT_FAST_N) trip(F_OCP_OUT_F, iout); } else c_ioutf = 0;
        if (iin_avg  > lim.iin_s)  trip(F_OCP_IN_S,  iin_avg);
        if (iout_avg > lim.iout_s) trip(F_OCP_OUT_S, iout_avg);
        /* Vin */
        if (vin > lim.vin_max) trip(F_OV_IN, vin);
        if (pwm_en && st != ST_PWMTEST && vin < VIN_UV_RUN) trip(F_UV_IN, vin);
        /* suhu */
        if (ntc_check && ntc_bad && st != ST_PWMTEST) trip(F_NTC, (float)raw->temp);
        if (!ntc_bad && temp > lim.tmax) trip(F_OTP, temp);
        /* PWMTEST: relai harus terbuka -> tidak boleh ada arus/tegangan daya */
        if (st == ST_PWMTEST) {
            if (vo > SIM_REAL_VO_MAX || iin > PWMTEST_IIN_A) { if (++c_relay >= PWMTEST_IIN_N) trip(F_RELAY, vo > SIM_REAL_VO_MAX ? vo : iin); }
            else c_relay = 0;
            if (!pwmtest_force && vin > PWMTEST_VIN_MAX) trip(F_RELAY, vin);   /* PSU dinyalakan saat pwmtest */
        }
    } else {
        c_ovp = c_iinf = c_ioutf = c_adcsat = 0;
    }
    /* K2 lepas / sensor V_OUT putus: lompatan turun yang tidak mungkin secara fisik */
    if (k2 && (st == ST_ARMED || st == ST_RUN_OL || st == ST_RUN_CL)) {
        if (vo_lp - vo > VODROP_V) { if (++c_vodrop >= VODROP_N) trip(F_VO_PLAUS, vo); }
        else c_vodrop = 0;
        vo_lp += (vo - vo_lp) * (QBC_TS / VODROP_TAU_S);
    } else { vo_lp = vo; c_vodrop = 0; }
    /* plausibilitas sensor Vo (V_OUT setelah K2): hanya saat K1 & K2 tertutup dan sudah tunak */
    if ((st == ST_ARMED || st == ST_RUN_OL || st == ST_RUN_CL) && vin > 8.0f) {
        if (vo < VOPLAUS_RATIO * vin - VOPLAUS_OFFS_V) {
            if (++c_plaus >= VOPLAUS_MS * QBC_TICKS_PER_MS) trip(F_VO_PLAUS, vo);
        } else c_plaus = 0;
    } else c_plaus = 0;
    /* link USB */
    if (!link_ok) link_lost_ticks++; else link_lost_ticks = 0;
    if (link_stop && link_lost_ticks == LINK_LOSS_MS * QBC_TICKS_PER_MS &&
        st != ST_OFF && st != ST_FAULT && st != ST_DISARM) {
        ev_push(EV_INFO, I_LINKLOST, 0.0f);
        pwm_en = 0;
        if (st == ST_PWMTEST) set_state(ST_OFF); else set_state(ST_DISARM);
    }

    /* ---- 5) Mesin status ---- */
    switch (st) {
    case ST_OFF:
        pwm_en = 0; k1 = 0; k2 = 0; duty_out = 0.0f;
        break;

    case ST_ARMING:
        pwm_en = 0; duty_out = 0.0f;
        if (phase == 0 && st_ticks >= K1_SETTLE_MS * QBC_TICKS_PER_MS) {
            if (vin < VIN_MIN_ARM) { trip(F_ARM, vin); break; }   /* sumber jatuh saat K1 tutup */
            k2 = 1; phase = 1; st_ticks = 0;
            ev_push(EV_INFO, I_ARMSTEP, 2.0f);
        } else if (phase == 1 && st_ticks >= K2_SETTLE_MS * QBC_TICKS_PER_MS) {
            if (vo < VOPLAUS_RATIO * vin - VOPLAUS_OFFS_V) { trip(F_ARM, vo); break; }
            set_state(ST_ARMED);
        }
        break;

    case ST_ARMED:
        pwm_en = 0; duty_out = 0.0f;
        break;

    case ST_PWMTEST:
    case ST_RUN_OL: {
        float step = drate * QBC_TS;
        float dmax = (st == ST_PWMTEST) ? PWMTEST_DMAX : ol_dmax;
        if (duty_cmd > dmax) duty_cmd = dmax;
        if (duty_now < duty_cmd) { duty_now += step; if (duty_now > duty_cmd) duty_now = duty_cmd; }
        else if (duty_now > duty_cmd) { duty_now -= step; if (duty_now < duty_cmd) duty_now = duty_cmd; }
        duty_out = duty_now;
        pwm_en = 1;
        if (st == ST_RUN_OL && noload_chk && vo > NOLOAD_VO_V && iout < NOLOAD_IOUT_A) {
            if (++c_noload >= NOLOAD_MS * QBC_TICKS_PER_MS) trip(F_NOLOAD, iout);
        } else c_noload = 0;
        break;
    }

    case ST_RUN_CL: {
        float step = vrate * QBC_TS;
        if (vref_now < vref_target)      { vref_now += step; if (vref_now > vref_target) vref_now = vref_target; }
        else if (vref_now > vref_target) { vref_now -= step; if (vref_now < vref_target) vref_now = vref_target; }
        duty_out = ctrl_step(vref_now, vo);
        pwm_en = 1;
        if (ctrl_last_sat() > 0) {
            sat_ticks_total++;
            c_dsat += 2u;
            if (c_dsat >= 2u * lim.dsat_ms * QBC_TICKS_PER_MS) trip(F_DSAT, duty_out);
        } else if (c_dsat) c_dsat--;
        break;
    }

    case ST_DISARM:
    case ST_FAULT:
        /* urutan buka: PWM mati -> 20 ms -> K1 -> (Vo < 30 V | Io < 0,1 A | 5 s) -> K2 */
        pwm_en = 0; duty_out = 0.0f;
        if (phase == 0) {
            if (!k1 && !k2) { phase = 2; }
            else if (st_ticks >= K1_OPEN_DELAY_MS * QBC_TICKS_PER_MS) { k1 = 0; phase = 1; st_ticks = 0; }
        } else if (phase == 1) {
            if (!k2 || vo < K2_OPEN_VO_V || iout < K2_OPEN_IO_A || st_ticks >= K2_OPEN_TMO_MS * QBC_TICKS_PER_MS) {
                if (k2 && vo > 50.0f) ev_push(EV_INFO, I_K2OPEN_HV, vo);   /* C13 masih bermuatan! */
                vo_at_k2open = vo;
                k2 = 0; phase = 2; st_ticks = 0;
            }
        } else {
            k1 = 0; k2 = 0;
            if (st == ST_DISARM) set_state(ST_OFF);
        }
        break;

    default:
        trip(F_USER, 0.0f);
        break;
    }
    if (!pwm_state(st)) pwm_en = 0;
    if (!pwm_en) duty_out = 0.0f;

    /* ---- 6) Kipas, trigger, titik cuplik ---- */
    if (fan_mode == 1) fan = 1;
    else if (fan_mode == 2) fan = 0;
    else {
        if (pwm_en || (!ntc_bad && temp > TEMP_FAN_ON)) fan = 1;
        else if (!pwm_en && (ntc_bad || temp < TEMP_FAN_OFF)) fan = 0;
    }
    if (trig_left) { trig_left--; trig_level = 1; } else trig_level = 0;

    if (pwm_en) {
        if (duty_out >= SAMPLE_MIDON_ABOVE) sample_midoff = 0;
        else if (duty_out <= SAMPLE_MIDOFF_BELOW) sample_midoff = 1;
        t_sample_us = sample_midoff ? (1.0f + duty_out) * 50.0f : duty_out * 50.0f;
        t_sample_us = clampf(t_sample_us, SAMPLE_T_MIN_US, SAMPLE_T_MAX_US);
    } else t_sample_us = 50.0f;

    /* ---- 7) Capture (satuan teknik, tidak tergantung kalibrasi saat dump) ---- */
    if (cap_state == CAP_ARMED || cap_state == CAP_POST) {
        if (++cap_dec_cnt >= cap_dec) {
            cap_t *p = &capbuf[cap_wr];
            cap_dec_cnt = 0;
            p->vo   = (uint16_t)clampf(vo * 100.0f, 0.0f, 65535.0f);
            p->vin  = (uint16_t)clampf(vin * 1000.0f, 0.0f, 65535.0f);
            p->iout = (uint16_t)clampf(iout * 10000.0f, 0.0f, 65535.0f);
            p->duty = (uint16_t)clampf(duty_out * 65535.0f, 0.0f, 65535.0f);
            cap_wr = (cap_wr + 1u) % CAP_N;
            if (cap_filled < CAP_N) cap_filled++;
            if (cap_state == CAP_POST) {
                if (cap_post_left) cap_post_left--;
                if (!cap_post_left) { cap_state = CAP_DONE; ev_push(EV_INFO, I_CAPDONE, (float)cap_filled); }
            }
        }
    }

    /* ---- 8) Keluaran (relai fisik dipaksa terbuka saat SIM) ---- */
    out->duty   = duty_out;
    out->pwm_en = (uint8_t)pwm_en;
    out->k1     = (uint8_t)(sim_on ? 0 : k1);
    out->k2     = (uint8_t)(sim_on ? 0 : k2);
    out->fan    = (uint8_t)fan;
    out->trig   = (uint8_t)trig_level;
    out->t_sample_us = t_sample_us;
}

/* ======================================================================================
 *  API loop
 * ====================================================================================== */
void qbc_set_link(int connected) { link_ok = connected ? 1 : 0; }
void qbc_external_fault(uint32_t f) { ext_fault |= f; }
void qbc_hw_stats(uint32_t isr_cycles, uint32_t overruns, uint32_t sysclk_hz, int hse)
{
    hw_isr_cyc = isr_cycles;
    if (isr_cycles > hw_isr_cyc_max) hw_isr_cyc_max = isr_cycles;
    hw_ovr = overruns; hw_sysclk = sysclk_hz; hw_hse = hse;
}
int      qbc_state(void)    { return st; }
uint32_t qbc_faults(void)   { return faults; }
float    qbc_vref_now(void) { return vref_now; }
float    qbc_duty(void)     { return duty_out; }
uint32_t qbc_ticks(void)    { return tick; }

const char *qbc_state_name(int s)
{
    static const char *const N[] = { "OFF", "ARMING", "ARMED", "RUN_OL", "RUN_CL", "PWMTEST", "DISARM", "FAULT" };
    return (s >= 0 && s <= 7) ? N[s] : "?";
}
const char *qbc_fault_name(int bit)
{
    static const char *const N[F_NFAULT] = {
        "OVP", "OCP_IN_F", "OCP_IN_S", "OCP_OUT_F", "OCP_OUT_S", "UV_IN", "OV_IN", "OTP", "NTC",
        "VO_PLAUS", "DSAT", "NOLOAD", "ARM", "USER", "ISR_STALL", "SIM_REAL", "ADC_SAT", "RELAY"
    };
    return (bit >= 0 && bit < F_NFAULT) ? N[bit] : "?";
}

/* ---------- format angka tanpa printf float (newlib-nano) ---------- */
static int put_s(char *b, int n, int pos, const char *s)
{
    while (*s && pos < n - 1) b[pos++] = *s++;
    b[pos] = 0;
    return pos;
}
static int put_u(char *b, int n, int pos, uint32_t v)
{
    char t[11]; int k = 0;
    do { t[k++] = (char)('0' + v % 10u); v /= 10u; } while (v && k < 10);
    while (k && pos < n - 1) b[pos++] = t[--k];
    b[pos] = 0;
    return pos;
}
static int put_i(char *b, int n, int pos, int32_t v)
{
    if (v < 0) { pos = put_s(b, n, pos, "-"); return put_u(b, n, pos, (uint32_t)(-v)); }
    return put_u(b, n, pos, (uint32_t)v);
}
static int put_f(char *b, int n, int pos, float v, int dec)
{
    uint32_t sc = 1, ip, fp, t;
    int i;
    char tmp[10];
    if (v != v) return put_s(b, n, pos, "nan");
    if (v < 0.0f) { pos = put_s(b, n, pos, "-"); v = -v; }
    if (v > 4.0e8f) return put_s(b, n, pos, "ovf");
    for (i = 0; i < dec; i++) sc *= 10u;
    t = (uint32_t)(v * (float)sc + 0.5f);
    ip = t / sc; fp = t % sc;
    pos = put_u(b, n, pos, ip);
    if (dec > 0) {
        pos = put_s(b, n, pos, ".");
        for (i = dec - 1; i >= 0; i--) { tmp[i] = (char)('0' + fp % 10u); fp /= 10u; }
        for (i = 0; i < dec && pos < n - 1; i++) b[pos++] = tmp[i];
        b[pos] = 0;
    }
    return pos;
}
/* angka kecil (gain) dalam notasi ilmiah sederhana */
static int put_e(char *b, int n, int pos, float v)
{
    int e = 0;
    if (v == 0.0f) return put_s(b, n, pos, "0");
    if (v < 0.0f) { pos = put_s(b, n, pos, "-"); v = -v; }
    while (v >= 10.0f) { v /= 10.0f; e++; }
    while (v < 1.0f)   { v *= 10.0f; e--; }
    pos = put_f(b, n, pos, v, 6);
    pos = put_s(b, n, pos, "e");
    return put_i(b, n, pos, e);
}

static int fmt_faults(char *b, int n, int pos, uint32_t f)
{
    int i, any = 0;
    for (i = 0; i < F_NFAULT; i++) {
        if (f & (1u << i)) {
            if (any) pos = put_s(b, n, pos, "|");
            pos = put_s(b, n, pos, qbc_fault_name(i));
            any = 1;
        }
    }
    if (!any) pos = put_s(b, n, pos, "-");
    return pos;
}

int qbc_poll_event(char *b, int n)
{
    static const char *const RS[] = { "", "status_tidak_sesuai", "Vin_di_luar_jendela",
        "ada_tegangan_di_sisi_beban", "NTC_tidak_terbaca", "suhu_terlalu_tinggi", "fault_belum_hilang",
        "vref_belum_diset(v <V>)", "relai_belum_selesai_dibuka", "ADC_asli_melihat_daya", "PWM_masih_jalan",
        "link_USB_tidak_aktif", "Vin_ada_saat_pwmtest(matikan_PSU_atau_pwmtest_force)" };
    static const char *const IS[] = { "", "TRIG", "CTRL", "CAPTURE_SELESAI", "K2_DIBUKA_C13_MASIH_BERMUATAN",
        "LINK_USB_PUTUS->DISARM", "SIM", "AW", "STEP", "ARM_LANGKAH" };
    ev_t e;
    int pos = 0;
    if (ev_tail == ev_head) return 0;
    e = evq[ev_tail];
    ev_tail = (ev_tail + 1u) % EV_N;
    pos = put_s(b, n, pos, "E,");
    pos = put_u(b, n, pos, e.tick / QBC_TICKS_PER_MS);
    pos = put_s(b, n, pos, ",");
    switch (e.type) {
    case EV_STATE:
        pos = put_s(b, n, pos, "STATE,"); pos = put_s(b, n, pos, qbc_state_name(e.code));
        if (e.code == ST_FAULT) { pos = put_s(b, n, pos, ","); pos = fmt_faults(b, n, pos, faults); }
        break;
    case EV_FAULT:
        pos = put_s(b, n, pos, "FAULT,"); pos = put_s(b, n, pos, qbc_fault_name(e.code));
        pos = put_s(b, n, pos, ","); pos = put_f(b, n, pos, e.val, 3);
        break;
    case EV_REFUSE:
        pos = put_s(b, n, pos, "TOLAK,");
        pos = put_s(b, n, pos, (e.code < sizeof(RS) / sizeof(RS[0])) ? RS[e.code] : "?");
        break;
    default:
        pos = put_s(b, n, pos, "INFO,");
        pos = put_s(b, n, pos, (e.code < sizeof(IS) / sizeof(IS[0])) ? IS[e.code] : "?");
        pos = put_s(b, n, pos, ","); pos = put_f(b, n, pos, e.val, 2);
        break;
    }
    if (ev_drop) { pos = put_s(b, n, pos, " (event hilang:"); pos = put_u(b, n, pos, ev_drop); pos = put_s(b, n, pos, ")"); ev_drop = 0; }
    pos = put_s(b, n, pos, "\r\n");
    return pos;
}

/* T,ms,state,ctrl,flags,vref,vo,vin,iin,iout,temp,duty,vo_min,vo_max,faults,isr_us,isr_us_max,ctrl_cyc,ovr,sat_ms */
int qbc_poll_telemetry(uint32_t now_ms, char *b, int n)
{
    static uint32_t last = 0;
    int pos = 0;
    float k;
    uint32_t flags, mhz;
    if (tel_ms == 0) return 0;
    if (!snap_ready) {
        if ((uint32_t)(now_ms - last) >= tel_ms) { last = now_ms; snap_req = 1; }
        return 0;
    }
    snap_ready = 0;
    if (snap.n == 0) return 0;
    k = 1.0f / (float)snap.n;
    flags = (uint32_t)sim_on | ((uint32_t)ctrl_aw() << 1) | ((uint32_t)k1 << 2) | ((uint32_t)k2 << 3) |
            ((uint32_t)fan << 4) | ((uint32_t)pwm_en << 5) | ((uint32_t)link_ok << 6) | ((uint32_t)ntc_bad << 7);
    mhz = hw_sysclk ? hw_sysclk / 1000000u : 168u;
    pos = put_s(b, n, pos, "T,");  pos = put_u(b, n, pos, now_ms);
    pos = put_s(b, n, pos, ",");   pos = put_s(b, n, pos, qbc_state_name(st));
    pos = put_s(b, n, pos, ",");   pos = put_s(b, n, pos, ctrl_name(ctrl_selected()));
    pos = put_s(b, n, pos, ",");   pos = put_u(b, n, pos, flags);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, vref_now, 2);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, snap.vo * k, 2);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, snap.vin * k, 2);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, snap.iin * k, 3);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, snap.iout * k, 4);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, ntc_bad ? NAN : temp, 1);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, snap.duty * k, 4);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, snap.vomin, 2);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, snap.vomax, 2);
    pos = put_s(b, n, pos, ",");   pos = fmt_faults(b, n, pos, faults);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, (float)hw_isr_cyc / (float)mhz, 2);
    pos = put_s(b, n, pos, ",");   pos = put_f(b, n, pos, (float)hw_isr_cyc_max / (float)mhz, 2);
    pos = put_s(b, n, pos, ",");   pos = put_u(b, n, pos, ctrl_last_cycles());
    pos = put_s(b, n, pos, ",");   pos = put_u(b, n, pos, hw_ovr);
    pos = put_s(b, n, pos, ",");   pos = put_u(b, n, pos, sat_ticks_total / QBC_TICKS_PER_MS);
    pos = put_s(b, n, pos, "\r\n");
    return pos;
}

int qbc_poll_output(char *b, int n)
{
    int pos = 0;
    if (raw_ready) {
        static const char *const NM[5] = { "vo", "vin", "iin", "iout", "temp" };
        int i;
        float N = (float)raw_n;
        pos = put_s(b, n, pos, "R,n=");
        pos = put_u(b, n, pos, raw_n);
        for (i = 0; i < 5; i++) {
            float m = (float)rs[i] / N;
            float var = (float)rss[i] / N - m * m;
            pos = put_s(b, n, pos, ",");  pos = put_s(b, n, pos, NM[i]);
            pos = put_s(b, n, pos, "=");  pos = put_f(b, n, pos, m, 2);
            pos = put_s(b, n, pos, "/s="); pos = put_f(b, n, pos, var > 0.0f ? sqrtf(var) : 0.0f, 2);
            if (i < 4) {
                pos = put_s(b, n, pos, "/x=");
                pos = put_f(b, n, pos, cal[i == 0 ? CH_VO : i == 1 ? CH_VIN : i == 2 ? CH_IIN : CH_IOUT].a * m +
                                         cal[i == 0 ? CH_VO : i == 1 ? CH_VIN : i == 2 ? CH_IIN : CH_IOUT].b, 4);
            } else {
                pos = put_s(b, n, pos, "/T=");
                pos = put_f(b, n, pos, ntc_temp((uint16_t)(m + 0.5f)), 1);
            }
        }
        pos = put_s(b, n, pos, "\r\n");
        raw_target = 0; raw_ready = 0;
        return pos;
    }
    if (dump_active) {
        uint32_t start = (cap_filled < CAP_N) ? 0u : cap_wr;   /* sampel tertua */
        uint32_t i;
        if (dump_idx == 0) {
            pos = put_s(b, n, pos, "C,HDR,n=");     pos = put_u(b, n, pos, cap_filled);
            pos = put_s(b, n, pos, ",pre=");        pos = put_u(b, n, pos, cap_pre);
            pos = put_s(b, n, pos, ",dt_us=");      pos = put_u(b, n, pos, 100u * cap_dec);
            pos = put_s(b, n, pos, ",src=");        pos = put_u(b, n, pos, (uint32_t)cap_trig_src);
            pos = put_s(b, n, pos, ",ctrl=");       pos = put_s(b, n, pos, ctrl_name(ctrl_selected()));
            pos = put_s(b, n, pos, ",trig_ms=");    pos = put_u(b, n, pos, cap_trig_tick / QBC_TICKS_PER_MS);
            pos = put_s(b, n, pos, "\r\nC,i,t_ms,vo,vin,iout,duty\r\n");
        }
        if (dump_idx >= cap_filled) {
            pos = put_s(b, n, pos, "C,END\r\n");
            dump_active = 0; dump_idx = 0; cap_state = CAP_IDLE;
            return pos;
        }
        i = (start + dump_idx) % CAP_N;
        {
            const cap_t *p = &capbuf[i];
            /* t = 0 di sampel pemicu (sampel ke-pre) */
            float t_ms = ((float)dump_idx - (float)(cap_filled - (CAP_N - cap_pre))) * 0.1f * (float)cap_dec;
            pos = put_s(b, n, pos, "C,"); pos = put_u(b, n, pos, dump_idx);
            pos = put_s(b, n, pos, ","); pos = put_f(b, n, pos, t_ms, 1);
            pos = put_s(b, n, pos, ","); pos = put_f(b, n, pos, p->vo * 0.01f, 2);
            pos = put_s(b, n, pos, ","); pos = put_f(b, n, pos, p->vin * 0.001f, 3);
            pos = put_s(b, n, pos, ","); pos = put_f(b, n, pos, p->iout * 0.0001f, 4);
            pos = put_s(b, n, pos, ","); pos = put_f(b, n, pos, p->duty / 65535.0f, 5);
            pos = put_s(b, n, pos, "\r\n");
        }
        dump_idx++;
        return pos;
    }
    return 0;
}

/* ======================================================================================
 *  Parser perintah (konteks loop)
 * ====================================================================================== */
static int streq(const char *a, const char *b) { return strcmp(a, b) == 0; }

static int set_lim(const char *name, float v, char *r, int n)
{
    /* hanya boleh mengetatkan: nilai <= batas keras (ambang atas) */
    struct { const char *nm; float *p; float lo, hi; } T[] = {
        { "ovp",    &lim.ovp,     5.0f,  OVP_V_HARD },
        { "iinf",   &lim.iin_f,   0.2f,  IIN_FAST_HARD },
        { "iins",   &lim.iin_s,   0.2f,  IIN_SLOW_HARD },
        { "ioutf",  &lim.iout_f,  0.05f, IOUT_FAST_HARD },
        { "iouts",  &lim.iout_s,  0.05f, IOUT_SLOW_HARD },
        { "vinmax", &lim.vin_max, 6.0f,  VIN_MAX_HARD },
        { "tmax",   &lim.tmax,    25.0f, TEMP_TRIP_HARD },
    };
    unsigned i;
    int pos = 0;
    if (streq(name, "dsat")) {
        if (v < 1.0f || v > (float)DSAT_MS_DEFAULT * 4.0f) { put_s(r, n, 0, "ERR dsat 1..200 ms\r\n"); return 1; }
        lim.dsat_ms = (uint32_t)v;
        pos = put_s(r, n, 0, "OK dsat="); pos = put_u(r, n, pos, lim.dsat_ms); put_s(r, n, pos, " ms\r\n");
        return 1;
    }
    for (i = 0; i < sizeof(T) / sizeof(T[0]); i++) {
        if (streq(name, T[i].nm)) {
            if (v < T[i].lo || v > T[i].hi) {
                pos = put_s(r, n, 0, "ERR "); pos = put_s(r, n, pos, name); pos = put_s(r, n, pos, " harus ");
                pos = put_f(r, n, pos, T[i].lo, 2); pos = put_s(r, n, pos, "..");
                pos = put_f(r, n, pos, T[i].hi, 2); put_s(r, n, pos, " (batas keras)\r\n");
                return 1;
            }
            *T[i].p = v;
            pos = put_s(r, n, 0, "OK "); pos = put_s(r, n, pos, name); pos = put_s(r, n, pos, "=");
            pos = put_f(r, n, pos, v, 3); put_s(r, n, pos, "\r\n");
            return 1;
        }
    }
    put_s(r, n, 0, "ERR nama: ovp iinf iins ioutf iouts vinmax tmax dsat\r\n");
    return 1;
}

static int print_lim(char *r, int n)
{
    int pos = 0;
    pos = put_s(r, n, pos, "LIM ovp=");   pos = put_f(r, n, pos, lim.ovp, 1);
    pos = put_s(r, n, pos, " iinf=");     pos = put_f(r, n, pos, lim.iin_f, 2);
    pos = put_s(r, n, pos, " iins=");     pos = put_f(r, n, pos, lim.iin_s, 2);
    pos = put_s(r, n, pos, " ioutf=");    pos = put_f(r, n, pos, lim.iout_f, 2);
    pos = put_s(r, n, pos, " iouts=");    pos = put_f(r, n, pos, lim.iout_s, 2);
    pos = put_s(r, n, pos, " vinmax=");   pos = put_f(r, n, pos, lim.vin_max, 1);
    pos = put_s(r, n, pos, " tmax=");     pos = put_f(r, n, pos, lim.tmax, 1);
    pos = put_s(r, n, pos, " dsat=");     pos = put_u(r, n, pos, lim.dsat_ms);
    pos = put_s(r, n, pos, "ms ntc=");    pos = put_u(r, n, pos, (uint32_t)ntc_check);
    pos = put_s(r, n, pos, " noload=");   pos = put_u(r, n, pos, (uint32_t)noload_chk);
    pos = put_s(r, n, pos, " link=");     pos = put_s(r, n, pos, link_stop ? "stop" : "keep");
    put_s(r, n, pos, "\r\n");
    return 1;
}

static int print_cfg(char *r, int n)
{
    int pos = 0, i;
    float kp, ki;
    pos = put_s(r, n, pos, "CFG " QBC_FW_VERSION " sysclk=");
    pos = put_u(r, n, pos, hw_sysclk);
    pos = put_s(r, n, pos, hw_hse == 1 ? "(HSE)" : hw_hse == 0 ? "(HSI!)" : "(?)");
    pos = put_s(r, n, pos, " ctrl=");  pos = put_s(r, n, pos, ctrl_name(ctrl_selected()));
    pos = put_s(r, n, pos, " aw=");    pos = put_u(r, n, pos, (uint32_t)ctrl_aw());
    pos = put_s(r, n, pos, " dcl=");   pos = put_f(r, n, pos, DUTY_CL_MIN, 2);
    pos = put_s(r, n, pos, "..");      pos = put_f(r, n, pos, DUTY_CL_MAX, 2);
    pos = put_s(r, n, pos, " dmax_ol="); pos = put_f(r, n, pos, ol_dmax, 3);
    pos = put_s(r, n, pos, " dr=");    pos = put_f(r, n, pos, drate, 3);
    pos = put_s(r, n, pos, " vref=");  pos = put_f(r, n, pos, vref_target, 2);
    pos = put_s(r, n, pos, " vr=");    pos = put_f(r, n, pos, vrate, 1);
    pos = put_s(r, n, pos, " tel=");   pos = put_u(r, n, pos, tel_ms);
    pos = put_s(r, n, pos, "ms sim="); pos = put_u(r, n, pos, (uint32_t)sim_on);
    if (sim_on) {
        pos = put_s(r, n, pos, "(R="); pos = put_f(r, n, pos, sim_R, 1);
        pos = put_s(r, n, pos, ",vin="); pos = put_f(r, n, pos, sim_vin, 2);
        pos = put_s(r, n, pos, sim_hw ? ",par=hw)" : ",par=model)");
    }
    pos = put_s(r, n, pos, "\r\nCAL");
    for (i = 0; i < CH_NCAL; i++) {
        pos = put_s(r, n, pos, " "); pos = put_s(r, n, pos, CH_NAME[i]); pos = put_s(r, n, pos, "=");
        pos = put_e(r, n, pos, cal[i].a); pos = put_s(r, n, pos, ","); pos = put_f(r, n, pos, cal[i].b, 4);
    }
    pos = put_s(r, n, pos, "\r\nGAIN");
    for (i = CTRL_PI; i <= CTRL_MPSO; i++) {
        ctrl_gains(i, &kp, &ki);
        pos = put_s(r, n, pos, " "); pos = put_s(r, n, pos, ctrl_name(i));
        pos = put_s(r, n, pos, ":Kp="); pos = put_e(r, n, pos, kp);
        pos = put_s(r, n, pos, ",Ki="); pos = put_e(r, n, pos, ki);
    }
    pos = put_s(r, n, pos, " HINF:orde6(controller_coeffs.h)\r\n");
    (void)pos;
    return 1;
}

static int print_status(char *r, int n)
{
    int pos = 0;
    pos = put_s(r, n, pos, "ST state=");  pos = put_s(r, n, pos, qbc_state_name(st));
    pos = put_s(r, n, pos, " ctrl=");     pos = put_s(r, n, pos, ctrl_name(ctrl_selected()));
    pos = put_s(r, n, pos, " vo=");       pos = put_f(r, n, pos, vo, 2);
    pos = put_s(r, n, pos, " vin=");      pos = put_f(r, n, pos, vin, 2);
    pos = put_s(r, n, pos, " iin=");      pos = put_f(r, n, pos, iin, 3);
    pos = put_s(r, n, pos, " iout=");     pos = put_f(r, n, pos, iout, 4);
    pos = put_s(r, n, pos, " T=");        pos = put_f(r, n, pos, ntc_bad ? NAN : temp, 1);
    pos = put_s(r, n, pos, " d=");        pos = put_f(r, n, pos, duty_out, 4);
    pos = put_s(r, n, pos, " vref=");     pos = put_f(r, n, pos, vref_now, 2);
    pos = put_s(r, n, pos, "/");          pos = put_f(r, n, pos, vref_target, 2);
    pos = put_s(r, n, pos, " K1=");       pos = put_u(r, n, pos, (uint32_t)k1);
    pos = put_s(r, n, pos, " K2=");       pos = put_u(r, n, pos, (uint32_t)k2);
    pos = put_s(r, n, pos, " fan=");      pos = put_u(r, n, pos, (uint32_t)fan);
    pos = put_s(r, n, pos, " sim=");      pos = put_u(r, n, pos, (uint32_t)sim_on);
    pos = put_s(r, n, pos, " link=");     pos = put_u(r, n, pos, (uint32_t)link_ok);
    pos = put_s(r, n, pos, " fault=");    pos = fmt_faults(r, n, pos, faults);
    pos = put_s(r, n, pos, " first=");    pos = fmt_faults(r, n, pos, fault_first);
    pos = put_s(r, n, pos, " ovr=");      pos = put_u(r, n, pos, hw_ovr);
    pos = put_s(r, n, pos, " cap=");      pos = put_u(r, n, pos, (uint32_t)cap_state);
    put_s(r, n, pos, "\r\n");
    return 1;
}

static const char HELP[] =
    "Perintah (akhiri Enter). Baris diawali '!' = DARURAT (PWM mati + relai dibuka).\r\n"
    " st | cfg | lim | ver | help\r\n"
    " arm | disarm | stop (x) | clr\r\n"
    " pwmtest [force]      T1/T2: PWM dengan relai dipaksa terbuka (default: Vin harus < 2 V)\r\n"
    " ol | d <D> | dr <1/s> | dmax <D>      open-loop (T4)\r\n"
    " ctrl hinf|pi|mpso | aw on|off | v <V> | vr <V/s> | cl | s <dV>   closed-loop (T5/T6)\r\n"
    " trig | cap <pre%> <dec> | capgo | capdump\r\n"
    " raw <n> | cal <vo|vin|iin|iout> <a> <b>   kalibrasi (T3)\r\n"
    " lim <ovp|iinf|iins|ioutf|iouts|vinmax|tmax|dsat> <nilai> | limreset   (U10)\r\n"
    " ntc on|off | noload on|off | fan auto|on|off | link stop|keep | tel <ms>\r\n"
    " sim on|off | sim r <ohm> | sim vin <V> | sim par model|hw   (uji meja tanpa daya)\r\n";

int qbc_command(const char *line, char *r, int n)
{
    char buf[64], *tok[4];
    int nt = 0, i, pos = 0;
    float a = 0.0f;

    if (n > 0) r[0] = 0;
    while (*line == ' ' || *line == '\t') line++;
    if (line[0] == '!') {                         /* darurat: tanpa parsing lebih lanjut */
        cq_push(OP_KILL, 0.0f);
        put_s(r, n, 0, "OK DARURAT\r\n");
        return 1;
    }
    for (i = 0; line[i] && i < (int)sizeof(buf) - 1; i++) {
        char c = line[i];
        buf[i] = (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
    }
    buf[i] = 0;
    {
        char *p = buf;
        while (*p && nt < 4) {
            while (*p == ' ' || *p == '\t' || *p == ',') *p++ = 0;
            if (!*p) break;
            tok[nt++] = p;
            while (*p && *p != ' ' && *p != '\t' && *p != ',') p++;
        }
    }
    if (nt == 0) return 0;
    if (nt >= 2) a = (float)atof(tok[1]);

#define OKQ(op, val) do { if (!cq_push((op), (val))) { put_s(r, n, 0, "ERR antrean penuh\r\n"); return 1; } \
                          put_s(r, n, 0, "OK\r\n"); return 1; } while (0)

    if (streq(tok[0], "help") || streq(tok[0], "?")) { put_s(r, n, 0, HELP); return 1; }
    if (streq(tok[0], "ver")) { put_s(r, n, 0, QBC_FW_VERSION "\r\n"); return 1; }
    if (streq(tok[0], "st"))  return print_status(r, n);
    if (streq(tok[0], "cfg")) return print_cfg(r, n);
    if (streq(tok[0], "arm"))     OKQ(OP_ARM, 0.0f);
    if (streq(tok[0], "disarm"))  OKQ(OP_DISARM, 0.0f);
    if (streq(tok[0], "stop") || streq(tok[0], "x")) OKQ(OP_STOP, 0.0f);
    if (streq(tok[0], "clr"))     OKQ(OP_CLR, 0.0f);
    if (streq(tok[0], "pwmtest")) {
        int force = (nt >= 2 && streq(tok[1], "force"));
        if (nt >= 2 && !force) { put_s(r, n, 0, "ERR pwmtest | pwmtest force\r\n"); return 1; }
        if (!cq_push(OP_PWMTEST, force ? 1.0f : 0.0f)) { put_s(r, n, 0, "ERR antrean penuh\r\n"); return 1; }
        put_s(r, n, 0, force ? "OK (force: Vin boleh ada, PASTIKAN K1 tidak lengket)\r\n" : "OK\r\n");
        return 1;
    }
    if (streq(tok[0], "ol"))      OKQ(OP_OL, 0.0f);
    if (streq(tok[0], "cl"))      OKQ(OP_CL, 0.0f);
    if (streq(tok[0], "trig"))    OKQ(OP_TRIG, 0.0f);
    if (streq(tok[0], "capgo"))   OKQ(OP_CAPGO, 0.0f);
    if (streq(tok[0], "d")) {
        if (nt < 2 || a < 0.0f || a > PWMTEST_DMAX) { put_s(r, n, 0, "ERR d 0..0.85 (OL dibatasi dmax)\r\n"); return 1; }
        OKQ(OP_DUTY, a);
    }
    if (streq(tok[0], "dr")) {
        if (nt < 2 || a <= 0.0f || a > OL_DRATE_HARD) { put_s(r, n, 0, "ERR dr 0..2 /s\r\n"); return 1; }
        drate = a; put_s(r, n, 0, "OK\r\n"); return 1;
    }
    if (streq(tok[0], "dmax")) {
        if (nt < 2 || a < 0.0f || a > OL_DMAX_HARD) { put_s(r, n, 0, "ERR dmax 0..0.75\r\n"); return 1; }
        ol_dmax = a; put_s(r, n, 0, "OK\r\n"); return 1;
    }
    if (streq(tok[0], "v")) {
        if (nt < 2 || a < 0.0f || a > VREF_MAX_HARD) { put_s(r, n, 0, "ERR v 0..160 V\r\n"); return 1; }
        OKQ(OP_VREF, a);
    }
    if (streq(tok[0], "vr")) {
        if (nt < 2 || a <= 0.0f || a > VREF_RATE_HARD) { put_s(r, n, 0, "ERR vr 0..6000 V/s\r\n"); return 1; }
        vrate = a;
        put_s(r, n, 0, a > 1000.0f ? "OK (PERINGATAN: ramp cepat menaikkan arus start)\r\n" : "OK\r\n");
        return 1;
    }
    if (streq(tok[0], "s")) {
        if (nt < 2 || a < -20.0f || a > 20.0f) { put_s(r, n, 0, "ERR s -20..20 V\r\n"); return 1; }
        OKQ(OP_STEP, a);
    }
    if (streq(tok[0], "ctrl")) {
        int w = -1;
        if (nt >= 2) {
            if (streq(tok[1], "hinf")) w = CTRL_HINF;
            else if (streq(tok[1], "pi")) w = CTRL_PI;
            else if (streq(tok[1], "mpso")) w = CTRL_MPSO;
        }
        if (w < 0) { put_s(r, n, 0, "ERR ctrl hinf|pi|mpso\r\n"); return 1; }
        OKQ(OP_CTRL, (float)w);
    }
    if (streq(tok[0], "aw")) {
        if (nt < 2 || (!streq(tok[1], "on") && !streq(tok[1], "off"))) { put_s(r, n, 0, "ERR aw on|off\r\n"); return 1; }
        OKQ(OP_AW, streq(tok[1], "on") ? 1.0f : 0.0f);
    }
    if (streq(tok[0], "raw")) {
        uint32_t k = (nt >= 2) ? (uint32_t)atol(tok[1]) : 1000u;
        if (k < 1u || k > 65000u) { put_s(r, n, 0, "ERR raw 1..65000\r\n"); return 1; }
        if (raw_target) { put_s(r, n, 0, "ERR raw sedang berjalan\r\n"); return 1; }
        memset(rs, 0, sizeof(rs)); memset(rss, 0, sizeof(rss)); raw_n = 0; raw_ready = 0;
        raw_target = k;
        put_s(r, n, 0, "OK (hasil di baris R,...)\r\n");
        return 1;
    }
    if (streq(tok[0], "cal")) {
        int ch = -1;
        for (i = 0; i < CH_NCAL && nt >= 2; i++) if (streq(tok[1], CH_NAME[i])) ch = i;
        if (ch < 0 || nt < 4) { put_s(r, n, 0, "ERR cal vo|vin|iin|iout <a> <b>\r\n"); return 1; }
        if (st != ST_OFF) { put_s(r, n, 0, "ERR cal hanya saat OFF\r\n"); return 1; }
        {
            float ca = (float)atof(tok[2]), cb = (float)atof(tok[3]);
            float nom = (ch == CH_VO) ? CAL_VO_A : (ch == CH_VIN) ? CAL_VIN_A : (ch == CH_IIN) ? CAL_IIN_A : CAL_IOUT_A;
            if (!(ca > 0.5f * nom && ca < 1.5f * nom)) { put_s(r, n, 0, "ERR a menyimpang >50% dari skematik, cek ulang\r\n"); return 1; }
            cal[ch].a = ca; cal[ch].b = cb;
        }
        put_s(r, n, 0, "OK (tidak tersimpan setelah reset; salin ke qbc_config.h)\r\n");
        return 1;
    }
    if (streq(tok[0], "lim")) {
        if (nt == 1) return print_lim(r, n);
        if (nt < 3) { put_s(r, n, 0, "ERR lim <nama> <nilai>\r\n"); return 1; }
        return set_lim(tok[1], (float)atof(tok[2]), r, n);
    }
    if (streq(tok[0], "limreset")) { lim_defaults(); put_s(r, n, 0, "OK\r\n"); return 1; }
    if (streq(tok[0], "ntc") || streq(tok[0], "noload")) {
        int on;
        if (nt < 2 || (!streq(tok[1], "on") && !streq(tok[1], "off"))) { put_s(r, n, 0, "ERR on|off\r\n"); return 1; }
        on = streq(tok[1], "on");
        if (streq(tok[0], "ntc")) ntc_check = on; else noload_chk = on;
        put_s(r, n, 0, on ? "OK\r\n" : "OK (PERINGATAN: proteksi dimatikan)\r\n");
        return 1;
    }
    if (streq(tok[0], "fan")) {
        if (nt >= 2 && streq(tok[1], "auto")) fan_mode = 0;
        else if (nt >= 2 && streq(tok[1], "on")) fan_mode = 1;
        else if (nt >= 2 && streq(tok[1], "off")) fan_mode = 2;
        else { put_s(r, n, 0, "ERR fan auto|on|off\r\n"); return 1; }
        put_s(r, n, 0, "OK\r\n"); return 1;
    }
    if (streq(tok[0], "link")) {
        if (nt >= 2 && streq(tok[1], "stop")) link_stop = 1;
        else if (nt >= 2 && streq(tok[1], "keep")) link_stop = 0;
        else { put_s(r, n, 0, "ERR link stop|keep\r\n"); return 1; }
        put_s(r, n, 0, "OK\r\n"); return 1;
    }
    if (streq(tok[0], "tel")) {
        if (nt < 2 || a < 0.0f || a > 10000.0f || (a > 0.0f && a < 10.0f)) { put_s(r, n, 0, "ERR tel 0 | 10..10000 ms\r\n"); return 1; }
        tel_ms = (uint32_t)a; put_s(r, n, 0, "OK\r\n"); return 1;
    }
    if (streq(tok[0], "cap")) {
        float pre = (nt >= 2) ? a : 20.0f;
        uint32_t dec = (nt >= 3) ? (uint32_t)atol(tok[2]) : 1u;
        if (pre < 0.0f || pre > 90.0f || dec < 1u || dec > 100u) { put_s(r, n, 0, "ERR cap <pre 0..90 %> <dec 1..100>\r\n"); return 1; }
        if (dump_active) { put_s(r, n, 0, "ERR dump sedang berjalan\r\n"); return 1; }
        cap_state = CAP_IDLE;
        cap_pre = (uint32_t)(pre / 100.0f * (float)CAP_N);
        cap_dec = dec; cap_dec_cnt = 0; cap_wr = 0; cap_filled = 0;
        cap_state = CAP_ARMED;
        pos = put_s(r, n, 0, "OK capture siap, jendela ");
        pos = put_f(r, n, pos, (float)CAP_N * 0.1f * (float)dec, 1);
        put_s(r, n, pos, " ms\r\n");
        return 1;
    }
    if (streq(tok[0], "capdump")) {
        if (cap_state != CAP_DONE) { put_s(r, n, 0, "ERR capture belum selesai\r\n"); return 1; }
        dump_idx = 0; dump_active = 1;
        put_s(r, n, 0, "OK\r\n"); return 1;
    }
    if (streq(tok[0], "sim")) {
        if (nt >= 2 && (streq(tok[1], "on") || streq(tok[1], "off"))) OKQ(OP_SIM, streq(tok[1], "on") ? 1.0f : 0.0f);
        if (nt >= 3 && streq(tok[1], "r"))   { float v = (float)atof(tok[2]); if (v < 5.0f || v > 1e5f) { put_s(r, n, 0, "ERR r 5..1e5\r\n"); return 1; } sim_R = v; put_s(r, n, 0, "OK\r\n"); return 1; }
        if (nt >= 3 && streq(tok[1], "vin")) { float v = (float)atof(tok[2]); if (v < 0.0f || v > 30.0f) { put_s(r, n, 0, "ERR vin 0..30\r\n"); return 1; } sim_vin = v; put_s(r, n, 0, "OK\r\n"); return 1; }
        if (nt >= 3 && streq(tok[1], "par")) {
            if (st != ST_OFF) { put_s(r, n, 0, "ERR hanya saat OFF\r\n"); return 1; }
            if (streq(tok[2], "hw")) { qbc_plant_par_hw(&simp); sim_hw = 1; }
            else { qbc_plant_par_model(&simp); sim_hw = 0; }
            put_s(r, n, 0, "OK\r\n"); return 1;
        }
        put_s(r, n, 0, "ERR sim on|off | r <ohm> | vin <V> | par model|hw\r\n");
        return 1;
    }
    put_s(r, n, 0, "ERR perintah tidak dikenal (ketik help)\r\n");
    return 1;
#undef OKQ
}
