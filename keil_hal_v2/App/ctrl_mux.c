/* ctrl_mux.c -- lihat ctrl_mux.h.
 * Sengaja TIDAK meng-include controller_coeffs.h: makro CTRL_TS di sana bentrok dengan CTRL_TS di
 * pi_params.h. File ini hanya butuh prototipe controller_step(). */
#include "ctrl_mux.h"
#include "controller_step.h"
#include "pi_ctrl.h"
#include "pi_params.h"
#include "qbc_config.h"
#include <stdint.h>

extern uint32_t qbc_hw_cycles(void);   /* DWT->CYCCNT di board, 0 di PC */

static const pi_params_t PI_SETS[2] = {
    /* Kp, Ki, Ts, Dmin, Dmax, xI0 (= D_MIN, sama dengan blok Discrete PID / PIL), nama */
    { PI_KP,   PI_KI,   CTRL_TS, D_MIN, D_MAX, D_MIN, "PI-FR"   },
    { MPSO_KP, MPSO_KI, CTRL_TS, D_MIN, D_MAX, D_MIN, "MPSO-PI" }
};

static int        sel = CTRL_HINF;
static int        aw_on = 1;
static int        sat_state = 0;     /* saturasi keluaran langkah sebelumnya */
static unsigned   cyc = 0;
static pi_state_t pis;

void ctrl_select(int which)
{
    if (which < 0 || which >= CTRL_N) which = CTRL_HINF;
    sel = which;
    ctrl_reset();
}
int  ctrl_selected(void) { return sel; }
const char *ctrl_name(int which)
{
    switch (which) {
    case CTRL_HINF: return "HINF";
    case CTRL_PI:   return "PI";
    case CTRL_MPSO: return "MPSO";
    default:        return "?";
    }
}
void ctrl_set_aw(int on) { aw_on = on ? 1 : 0; }
int  ctrl_aw(void)       { return aw_on; }
int  ctrl_last_sat(void) { return sat_state; }
unsigned ctrl_last_cycles(void) { return cyc; }

void ctrl_gains(int which, float *kp, float *ki)
{
    float p = 0.0f, i = 0.0f;
    if (which == CTRL_PI)   { p = (float)PI_KP;   i = (float)PI_KI; }
    if (which == CTRL_MPSO) { p = (float)MPSO_KP; i = (float)MPSO_KI; }
    if (kp) *kp = p;
    if (ki) *ki = i;
}

void ctrl_reset(void)
{
    sat_state = 0;
    if (sel == CTRL_HINF) controller_init();
    else pi_reset(&PI_SETS[sel - 1], &pis);
}

float ctrl_step(float vref, float vo)
{
    float d;
    uint32_t t0 = qbc_hw_cycles();
    if (sel == CTRL_HINF) {
        float vr = vref;
        float e  = vref - vo;
        if (aw_on && ((sat_state > 0 && e > 0.0f) || (sat_state < 0 && e < 0.0f)))
            vr = vo;                                 /* galat diumpankan 0: state tidak di-"integrasi" */
        d = controller_step(vr, vo);                 /* jepit internal +-5 (file asli) */
        if (!(d == d)) d = DUTY_CL_MIN;              /* NaN -> duty minimum (dan DSAT/proteksi lain) */
        if (d > DUTY_CL_MAX)      { d = DUTY_CL_MAX; sat_state = 1; }
        else if (d < DUTY_CL_MIN) { d = DUTY_CL_MIN; sat_state = -1; }
        else sat_state = 0;
    } else {
        pi_real u = pi_step(&PI_SETS[sel - 1], &pis, (pi_real)vref - (pi_real)vo);
        d = (float)u;                                /* sudah 0,05..0,85 dengan clamping */
        if (!(d == d)) d = DUTY_CL_MIN;
        sat_state = (d >= DUTY_CL_MAX - 1e-6f) ? 1 : ((d <= DUTY_CL_MIN + 1e-6f) ? -1 : 0);
    }
    cyc = (unsigned)(qbc_hw_cycles() - t0);
    return d;
}
