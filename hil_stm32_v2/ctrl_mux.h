/* ctrl_mux.h -- pemilih pengendali H-inf / PI respons frekuensi / MPSO-PI.
 *
 * File pengendali yang sudah lolos SIL/PIL dipakai TANPA diubah:
 *   controller_step.c/.h, controller_coeffs.h   (H-inf orde 6, md5 sama dengan pil_stm32)
 *   pi_ctrl.c/.h, pi_params.h                   (PI & MPSO-PI, sama dengan PIL_PI_MPSO/3_controller_master)
 * Wrapper PI versi PIL (controller_step.c milik PI) TIDAK dipakai karena namanya bentrok dengan
 * controller_step H-inf; pi_step() dipanggil langsung dengan parameter dari pi_params.h.
 *
 * Saturasi dan anti-windup ada di sini (di luar file pengendali):
 *   - Semua pengendali dijepit DUTY_CL_MIN..DUTY_CL_MAX (0,05..0,85, sama dengan MIL-2/PIL).
 *   - PI/MPSO-PI: clamping Simulink sudah ada di pi_step().
 *   - H-inf: controller_step() hanya menjepit di +-5 dan state-nya tidak bisa diakses. Anti-windup
 *     dilakukan dengan "conditional integration" pada masukan: jika keluaran langkah sebelumnya
 *     tersaturasi dan galat searah dengan saturasi, galat yang diumpankan = 0 (vref = vo).
 *     Bisa dimatikan (aw off) untuk replikasi MIL-2 murni.
 */
#ifndef CTRL_MUX_H
#define CTRL_MUX_H
#ifdef __cplusplus
extern "C" {
#endif

enum { CTRL_HINF = 0, CTRL_PI = 1, CTRL_MPSO = 2, CTRL_N = 3 };

void        ctrl_select(int which);         /* hanya dipanggil saat PWM mati */
int         ctrl_selected(void);
const char *ctrl_name(int which);
void        ctrl_reset(void);               /* reset state pengendali terpilih */
float       ctrl_step(float vref, float vo); /* duty tersaturasi 0,05..0,85 */
void        ctrl_set_aw(int on);
int         ctrl_aw(void);
int         ctrl_last_sat(void);            /* +1 atas, -1 bawah, 0 tidak */
unsigned    ctrl_last_cycles(void);
void        ctrl_gains(int which, float *kp, float *ki);

#ifdef __cplusplus
}
#endif
#endif
