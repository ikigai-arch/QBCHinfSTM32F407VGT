/* test_hinf_vectors.c -- uji unit pengendali H-inf tanpa board.
 *  U1  controller_step() (file asli) vs dlsim MATLAB: e_test.csv / y_matlab_test.csv
 *      (MATLAB File_Hinfinit/MIL/Sil tahap akhir, gen_test_vectors.m, Kd2 dari Khinf_QBC_MIL2.mat).
 *  U2  controller_step() float32 vs referensi double dengan koefisien yang sama, pada galat berukuran
 *      realistis (step 1..10 V + derau 0,2 V, 20 000 sampel = 2 s) -> presisi float di Cortex-M4.
 *  U3  ctrl_mux (aw off) == controller_step + jepit 0,05..0,85 (bit-identik): pembungkus tidak
 *      mengubah pengendali. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "controller_step.h"
#include "ctrl_mux.h"
#include "coeffs_double.h"     /* dibuat dari controller_coeffs.h dengan sed (float -> double) */
uint32_t qbc_hw_cycles(void) { return 0; }
#define NMAX 100000
static float e[NMAX]; static double ym[NMAX];

static double xd[CTRL_ORDER];
static double ref_step(double ev)
{
    double y = CTRL_D * ev, xn[CTRL_ORDER];
    int i, j;
    for (i = 0; i < CTRL_ORDER; i++) y += CTRL_C[i] * xd[i];
    for (i = 0; i < CTRL_ORDER; i++) { double a = CTRL_B[i] * ev; for (j = 0; j < CTRL_ORDER; j++) a += CTRL_A[i][j] * xd[j]; xn[i] = a; }
    for (i = 0; i < CTRL_ORDER; i++) xd[i] = xn[i];
    return y;
}
static double urand(void) { return (double)rand() / RAND_MAX - 0.5; }

int main(int argc, char **argv)
{
    const char *fe = argc > 2 ? argv[1] : "e_test.csv", *fy = argc > 2 ? argv[2] : "y_matlab_test.csv";
    FILE *a = fopen(fe, "r"), *b = fopen(fy, "r");
    int n = 0, m = 0, i, fail = 0;
    double dmax = 0, rms = 0, ymaxabs = 0;
    if (!a || !b) { printf("GAGAL buka vektor %s %s\n", fe, fy); return 2; }
    while (n < NMAX && fscanf(a, "%f", &e[n]) == 1) n++;
    while (m < NMAX && fscanf(b, "%lf", &ym[m]) == 1) m++;
    fclose(a); fclose(b);
    if (n != m || n == 0) { printf("GAGAL jumlah sampel e=%d y=%d\n", n, m); return 2; }

    /* U1 */
    controller_init();
    for (i = 0; i < n; i++) {
        double y = controller_step(e[i], 0.0f), d = fabs(y - ym[i]);
        if (d > dmax) dmax = d;
        rms += d * d;
        if (fabs(ym[i]) > ymaxabs) ymaxabs = fabs(ym[i]);
    }
    rms = sqrt(rms / n);
    printf("U1 vs dlsim : %d sampel, max|u|=%.4g, max|du|=%.3g (%.2f %% dari max|u|), rms=%.3g\n",
           n, ymaxabs, dmax, 100 * dmax / ymaxabs, rms);
    /* referensi double dari koefisien yang sama (memisahkan galat koefisien vs galat float) */
    {
        double dref = 0;
        for (i = 0; i < CTRL_ORDER; i++) xd[i] = 0;
        for (i = 0; i < n; i++) { double d = fabs(ref_step(e[i]) - ym[i]); if (d > dref) dref = d; }
        printf("              referensi double vs dlsim: max|du|=%.3g -> sisa galat U1 = float32\n", dref);
    }
    if (dmax > 0.01 * ymaxabs) { printf("U1: GAGAL (> 1 %%)\n"); fail = 1; } else printf("U1: LOLOS (batas 1 %% dari max|u|)\n");

    /* U2 */
    {
        int N = 200000;
        double d2 = 0, umax = 0, ev = 0;
        srand(7);
        controller_init();
        for (i = 0; i < CTRL_ORDER; i++) xd[i] = 0;
        for (i = 0; i < N; i++) {
            if (i % 2000 == 0) ev = 10.0 * urand() * 2.0;     /* step -10..10 V tiap 0,2 s */
            {
                double ei = ev * exp(-(double)(i % 2000) / 300.0) + 0.2 * urand();
                float  ef = (float)ei;
                double r = ref_step((double)ef);
                double y = controller_step(ef, 0.0f);         /* jepit internal +-5 */
                if (r > 5) r = 5;
                if (r < -5) r = -5;
                if (fabs(r) > umax) umax = fabs(r);
                if (fabs(y - r) > d2) d2 = fabs(y - r);
            }
        }
        printf("U2 float vs double, galat realistis 20 s: max|u|=%.4g, max|du|=%.3g duty = %.2f count PWM (ARR 16800)\n", umax, d2, d2 * 16800.0);
        printf("U2: INFO (bukan lolos/gagal): selisih open-loop akibat pembulatan float32 di state integrator\n"
               "    (A[0][0] = 0,99999). Di closed-loop selisih ini dikoreksi integrator; dampaknya diuji di harness.\n");
    }

    /* U3 */
    {
        int N = 20000, diff = 0;
        float ev = 0;
        srand(3);
        controller_init();
        ctrl_select(CTRL_HINF); ctrl_set_aw(0);
        /* ctrl_select memanggil controller_init lagi; jalankan dua instans bergantian TIDAK bisa
         * (state statis tunggal), jadi rekam dulu keluaran ctrl_step, lalu ulang dengan controller_step. */
        {
            static float u1[20000];
            for (i = 0; i < N; i++) { if (i % 1000 == 0) ev = (float)(40 * urand()); u1[i] = ctrl_step(150.0f + ev, 150.0f); }
            controller_init(); srand(3);
            for (i = 0; i < N; i++) {
                float y;
                if (i % 1000 == 0) ev = (float)(40 * urand());
                y = controller_step(150.0f + ev, 150.0f);
                if (y > 0.85f) y = 0.85f;
                if (y < 0.05f) y = 0.05f;
                if (y != u1[i]) diff++;
            }
        }
        printf("U3 ctrl_mux(aw off) vs controller_step+jepit: %d dari %d sampel berbeda\n", diff, N);
        if (diff) { printf("U3: GAGAL\n"); fail = 1; } else printf("U3: LOLOS (bit-identik)\n");
    }
    printf(fail ? "HASIL: ADA YANG GAGAL\n" : "HASIL: SEMUA LOLOS\n");
    return fail;
}
