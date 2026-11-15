/* qbc_sim.c -- lihat qbc_sim.h. Persamaan sama dengan qbc_avg_deriv8.m (QBC_MIL_4Ctrl):
 *   Mode ON  (S, D2 on) : L1: Vin -> L1 -> D2 -> S ;  L2: C1 -> L2 -> S
 *   Mode OFF (D1, D3 on): L1: Vin -> L1 -> D1 -> C1 ; L2: C1 -> L2 -> D3 -> C2
 * Penamaan model: D1 = D6, D2 = D7, D3 = D9 pada skematik KiCad. */
#include "qbc_sim.h"

void qbc_plant_par_model(qbc_plant_par_t *p)
{
    p->L1 = 1e-3f; p->L2 = 3e-3f; p->C1 = 100e-6f; p->C2 = 100e-6f;
    p->rL1 = 0.005f; p->rL2 = 0.01f; p->rC1 = 0.01f; p->rC2 = 0.01f;
    p->Ron = 0.1f; p->rD = 0.001f; p->VfD3 = 0.8f; p->VfD12 = 0.0f;
}

void qbc_plant_par_hw(qbc_plant_par_t *p)
{
    p->L1 = 1e-3f; p->L2 = 3e-3f; p->C1 = 100e-6f; p->C2 = 100e-6f;
    p->rL1 = 0.030f; p->rL2 = 0.080f; p->rC1 = 0.50f; p->rC2 = 0.50f;
    p->Ron = 0.078f; p->rD = 0.030f; p->VfD3 = 1.2f; p->VfD12 = 1.2f;
}

void qbc_plant_reset(qbc_plant_x_t *x)
{
    x->iL1 = 0.0f; x->iL2 = 0.0f; x->vC1 = 0.0f; x->vC2 = 0.0f;
}

float qbc_plant_step(const qbc_plant_par_t *p, qbc_plant_x_t *x, float d, float vin, float R, float dt)
{
    /* Euler semi-implisit (symplectic): arus induktor diperbarui dulu dengan tegangan lama, lalu
     * tegangan kapasitor dengan arus BARU. Euler eksplisit biasa pada dt = 10 us tidak stabil untuk
     * set 'model' (redaman kecil: rC = 10 mOhm) -> osilasi numerik +-5 V yang tidak ada di MIL.
     * Varian ini stabil pada dt = 10 us untuk kedua set (diuji di test_pc, pembanding dt = 0,1 us). */
    const float Rl = (R > 0.0f) ? R : 1e9f;          /* K2 terbuka = beban tak hingga */
    float iL1 = x->iL1, iL2 = x->iL2;
    const float vC1 = x->vC1, vC2 = x->vC2;
    float on1, on2, of1, of2, iC1, vo_on, vo_off, dp;
    float vf12 = (iL1 > 0.0f) ? p->VfD12 : 0.0f;     /* dioda tidak menghantar tanpa arus */
    float vf3  = (iL2 > 0.0f) ? p->VfD3  : 0.0f;

    if (d < 0.0f) d = 0.0f;
    if (d > 1.0f) d = 1.0f;
    dp = 1.0f - d;

    /* 1) arus induktor dari tegangan lama */
    iC1    = iL1 - iL2;
    vo_off = (vC2 + p->rC2 * iL2) * Rl / (Rl + p->rC2);
    on1 = (vin - (p->rL1 + p->rD) * iL1 - vf12 - p->Ron * (iL1 + iL2)) / p->L1;
    on2 = (vC1 - (p->rC1 + p->rL2) * iL2 - p->Ron * (iL1 + iL2)) / p->L2;
    of1 = (vin - (p->rL1 + p->rD) * iL1 - vf12 - vC1 - p->rC1 * iC1) / p->L1;
    of2 = (vC1 + p->rC1 * iC1 - (p->rL2 + p->rD) * iL2 - vf3 - vo_off) / p->L2;
    iL1 += dt * (d * on1 + dp * of1);
    iL2 += dt * (d * on2 + dp * of2);
    /* dioda: arus induktor tidak bisa negatif (pendekatan DCM) */
    if (iL1 < 0.0f) iL1 = 0.0f;
    if (iL2 < 0.0f) iL2 = 0.0f;

    /* 2) tegangan kapasitor dari arus baru */
    iC1    = iL1 - iL2;
    vo_on  = vC2 * Rl / (Rl + p->rC2);
    vo_off = (vC2 + p->rC2 * iL2) * Rl / (Rl + p->rC2);
    x->vC1 = vC1 + dt * (d * (-iL2) + dp * iC1) / p->C1;
    x->vC2 = vC2 + dt * (d * (-vo_on / Rl) + dp * (iL2 - vo_off / Rl)) / p->C2;
    if (x->vC1 < 0.0f) x->vC1 = 0.0f;
    if (x->vC2 < 0.0f) x->vC2 = 0.0f;
    x->iL1 = iL1;
    x->iL2 = iL2;
    return d * vo_on + dp * vo_off;
}
