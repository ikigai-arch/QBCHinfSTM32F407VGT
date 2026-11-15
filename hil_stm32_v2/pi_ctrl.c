/* pi_ctrl.c -- lihat pi_ctrl.h. Urutan operasi sengaja mengikuti blok Discrete PID Simulink. */
#include "pi_ctrl.h"

void pi_reset(const pi_params_t *p, pi_state_t *s)
{
    s->xI = p->xI0;
}

pi_real pi_step(const pi_params_t *p, pi_state_t *s, pi_real e)
{
    pi_real v  = p->Kp * e + s->xI;          /* keluaran sebelum saturasi */
    pi_real u  = v;
    pi_real iu = p->Ki * e;                  /* masukan integrator (sebelum dikali Ts) */
    int clamp  = 0;
    if (v > p->Dmax)      { u = p->Dmax; clamp = (iu > 0); }
    else if (v < p->Dmin) { u = p->Dmin; clamp = (iu < 0); }
    if (!clamp) s->xI = s->xI + p->Ts * iu;  /* Forward Euler: x[k+1] = x[k] + Ts*Ki*e[k] */
    return u;
}
