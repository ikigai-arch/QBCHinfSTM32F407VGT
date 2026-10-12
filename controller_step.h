#ifndef CONTROLLER_STEP_H
#define CONTROLLER_STEP_H

/* Kontroler H-infinity diskret (Ts = CTRL_TS), hasil c2d(Kr,'tustin')
 * dari skripsi QBC H-infinity PIL. Dipakai sama persis di MIL-2 (lewat
 * blok Discrete State-Space), SIL (lewat C Caller/Embedded Coder), dan
 * nanti PIL (STM32F407VGT).
 *
 * Alur pakai tiap periode Ts:
 *   duty = controller_step(vref, vo_meas);
 * vref dan vo_meas dalam VOLT (bukan count ADC) -- skala ADC->volt
 * dilakukan di luar fungsi ini, supaya file ini sama persis dari SIL
 * sampai PIL.
 */

void  controller_init(void);
float controller_step(float vref, float vo_meas);

#endif /* CONTROLLER_STEP_H */
