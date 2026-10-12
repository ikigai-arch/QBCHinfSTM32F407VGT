#include "controller_step.h"
#include "controller_coeffs.h"

/* State internal kontroler, x[k]. static supaya nilainya tetap antar
 * panggilan (persis seperti "Initial states" blok Discrete State-Space
 * di Simulink -- di sini otomatis 0 di awal, sama seperti IC=0 di sana). */
static float x[CTRL_ORDER];

void controller_init(void)
{
    int i;
    for (i = 0; i < CTRL_ORDER; i++) {
        x[i] = 0.0f;
    }
}

/* x[k+1] = A*x[k] + B*e[k]
 * y[k]   = C*x[k] + D*e[k]
 * duty   = saturate(y[k], DUTY_MIN, DUTY_MAX)
 *
 * PENTING: y dihitung dari x LAMA (sebelum update), baru setelah itu
 * x diperbarui -- urutan ini harus sama dengan Discrete State-Space
 * Simulink, jangan dibalik. Tidak ada anti-windup di sini (sengaja),
 * karena blok Saturation di MIL-2 juga tidak memakai anti-windup --
 * supaya SIL identik dengan MIL-2 untuk dibandingkan.
 */
float controller_step(float vref, float vo_meas)
{
    float e = vref - vo_meas;
    float y;
    float x_new[CTRL_ORDER];
    int i, j;

    /* y[k] = C*x[k] + D*e[k], pakai x LAMA */
    y = CTRL_D * e;
    for (i = 0; i < CTRL_ORDER; i++) {
        y += CTRL_C[i] * x[i];
    }

    /* x[k+1] = A*x[k] + B*e[k], juga pakai x LAMA (bukan x_new) */
    for (i = 0; i < CTRL_ORDER; i++) {
        float acc = CTRL_B[i] * e;
        for (j = 0; j < CTRL_ORDER; j++) {
            acc += CTRL_A[i][j] * x[j];
        }
        x_new[i] = acc;
    }
    for (i = 0; i < CTRL_ORDER; i++) {
        x[i] = x_new[i];
    }

    /* Saturasi duty, samakan dengan blok Saturation Simulink */
    if (y > DUTY_MAX) { y = DUTY_MAX; }
    if (y < DUTY_MIN) { y = DUTY_MIN; }

    return y;
}
