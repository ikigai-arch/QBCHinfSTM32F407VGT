/* qbc_sim.h -- model rata-rata QBC dengan 8 parasitik (port C dari qbc_avg_deriv8.m).
 * Dipakai di dua tempat:
 *   1) mode SIM di board (uji meja tanpa daya: relai fisik dipaksa terbuka);
 *   2) harness uji PC (test_pc/).
 * Model hanya untuk CCM; arus induktor dijepit >= 0 sebagai pendekatan kasar DCM.
 */
#ifndef QBC_SIM_H
#define QBC_SIM_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float L1, L2, C1, C2;
    float rL1, rL2, rC1, rC2, Ron, rD, VfD3, VfD12;
} qbc_plant_par_t;

typedef struct {
    float iL1, iL2, vC1, vC2;
} qbc_plant_x_t;

void  qbc_plant_par_model(qbc_plant_par_t *p);   /* parasitik set 'model' (MIL2.slx)        */
void  qbc_plant_par_hw(qbc_plant_par_t *p);      /* parasitik set 'hw' (perkiraan hardware) */
void  qbc_plant_reset(qbc_plant_x_t *x);
/* satu langkah Euler dt. vin = tegangan di belakang K1 (0 jika K1 terbuka),
 * R = beban di belakang K2 (<= 0 berarti terbuka). Mengembalikan Vo terminal (sisi C2). */
float qbc_plant_step(const qbc_plant_par_t *p, qbc_plant_x_t *x, float d, float vin, float R, float dt);

#ifdef __cplusplus
}
#endif
#endif
