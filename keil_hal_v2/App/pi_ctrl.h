#ifndef PI_CTRL_H
#define PI_CTRL_H
/* pi_ctrl.h -- PI diskret QBC, IDENTIK dengan blok "Discrete PID Controller" di MIL2_PI.slx /
 * MIL2_MPSOPI.slx:  Controller PI, Form Parallel, Integrator method Forward Euler, Ts = 100 us,
 * Initial condition integrator = D_MIN, Limit output D_MIN..D_MAX, Anti-windup = clamping.
 *
 *   u_presat[k] = Kp*e[k] + xI[k]
 *   u[k]        = sat(u_presat[k], D_MIN, D_MAX)
 *   xI[k+1]     = xI[k] + Ts*Ki*e[k]   KECUALI (clamping Simulink):
 *                 u_presat di luar batas DAN tanda (Ki*e) searah dengan pelanggarannya
 *
 * Tipe aritmetika: pi_real = double (bawaan) supaya integrator yang menjumlah ~1e4 langkah tetap
 * sama dengan Simulink (double). Definisikan PI_USE_FLOAT untuk versi float32 penuh (lebih cepat
 * di FPU Cortex-M4, tetapi akumulasi integrator menyimpang ~1e-5..1e-4 setelah >1 s).
 */
#ifdef PI_USE_FLOAT
typedef float  pi_real;
#else
typedef double pi_real;
#endif

typedef struct {
    pi_real Kp, Ki, Ts, Dmin, Dmax, xI0;
    const char *name;
} pi_params_t;

typedef struct {
    pi_real xI;
} pi_state_t;

void    pi_reset(const pi_params_t *p, pi_state_t *s);
pi_real pi_step(const pi_params_t *p, pi_state_t *s, pi_real e);

#endif
