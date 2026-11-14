/* test_controller_main.c
 * Uji unit controller_step(): baca e_test.csv (satu kolom, hasil
 * gen_test_vectors.m), panggil controller_step(e[k], 0) tiap sampel
 * (supaya e_dalam = vref - vo_meas = e[k] - 0 = e[k]), tulis
 * keluarannya ke y_c_test.csv untuk dibandingkan dengan dlsim MATLAB.
 *
 * Compile (contoh, pakai gcc / MinGW):
 *   gcc -O2 controller_step.c test_controller_main.c -o test_ctrl.exe
 * Jalankan:
 *   test_ctrl.exe
 */
#include <stdio.h>
#include <stdlib.h>
#include "controller_step.h"

#define MAX_SAMPLES 100000

int main(void)
{
    FILE *fin, *fout;
    float e[MAX_SAMPLES];
    int n = 0;

    fin = fopen("e_test.csv", "r");
    if (!fin) {
        fprintf(stderr, "Gagal buka e_test.csv (jalankan gen_test_vectors.m dulu)\n");
        return 1;
    }
    while (n < MAX_SAMPLES && fscanf(fin, "%f", &e[n]) == 1) {
        n++;
    }
    fclose(fin);

    fout = fopen("y_c_test.csv", "w");
    if (!fout) {
        fprintf(stderr, "Gagal buat y_c_test.csv\n");
        return 1;
    }

    controller_init();
    for (int i = 0; i < n; i++) {
        float duty = controller_step(e[i], 0.0f);   /* e_dalam = e[i] - 0 */
        fprintf(fout, "%.9g\n", duty);
    }
    fclose(fout);

    printf("Selesai: %d sampel diproses -> y_c_test.csv\n", n);
    return 0;
}
