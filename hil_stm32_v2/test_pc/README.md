# test_pc: uji firmware v2 tanpa board

Harness C biasa yang mengompilasi file firmware **apa adanya** dari folder sketch (`..`) dan
menjalankannya terhadap plant rata-rata QBC. Arduino IDE mengabaikan folder ini.

## Menjalankan

Butuh `gcc` dan `make` (MSYS2/MinGW, WSL, atau Linux):

```
cd hil_stm32_v2/test_pc
make test
```

`make test` menjalankan dua program:

1. `thv`: uji unit H∞
   - U1: `controller_step()` vs `dlsim` MATLAB dengan vektor `MATLAB File_Hinfinit/MIL/Sil tahap akhir/
     e_test.csv` dan `y_matlab_test.csv` (path diatur `VEC` di Makefile; `make test VEC=...` untuk
     lokasi lain).
   - U2: float32 vs double dengan koefisien yang sama pada galat realistis (info).
   - U3: pembungkus `ctrl_mux` (aw off) bit-identik dengan `controller_step` + jepit.
2. `harness`: 30 uji closed-loop/proteksi. Pilih sebagian dengan `./harness ovp s2_mil ...`.

`make warn` mengompilasi file portabel dengan peringatan penuh (termasuk `-Wdouble-promotion`).

## Model board di harness

- Plant: `qbc_sim.c` (port `qbc_avg_deriv8.m`), set parasitik `hw` (bawaan) atau `model`, 10
  sub-langkah Euler semi-implisit per Ts (akurasi dicek uji `plant_acc`).
- Relai K1 (sumber → converter) dan K2 (converter → beban); bisa dibuat lengket/lepas.
- Sensor seperti skematik: V_IN sebelum K1, V_OUT dan I_OUT setelah K2, I_IN = iL1. Kuantisasi 12 bit
  dengan skala `qbc_config.h` + derau ±1 LSB. NTC 10 k/B3950 dengan R11 10 k.
- Urutan per tick: cuplik → `qbc_isr()` → duty berlaku selama Ts berikutnya.
- Invarian keselamatan dicek di setiap tick: PWM mati di OFF/ARMING/ARMED/DISARM/FAULT, relai terbuka
  di OFF dan PWMTEST, duty CL 0,05–0,85, duty OL ≤ 0,75, K2 tidak tertutup tanpa K1, duty bukan NaN.
- Setiap uji = proses terpisah (state statis firmware bersih seperti setelah reset).

**Batasan:** model rata-rata (tanpa riak switching, dead-time, filter Sallen-Key, respons relai, timing
ISR nyata). `hw_f407.c` dan `.ino` tidak diuji di sini.

## Hasil (15 Nov 2026, v0.9.1)

`TOTAL: 30 uji, 193 cek, 0 gagal`; uji vektor H∞ `SEMUA LOLOS`.

- `hasil/hasil_uji.txt`: rincian setiap cek, perintah, dan event firmware.
- `hasil/ringkasan_stdout.txt`: ringkasan + semua baris info (angka prediksi).
- `hasil/hasil_hinf_vectors.txt`: U1–U3.
- `hasil/*.csv`: jejak waktu (t_ms, state, vref, Vo terminal, Vo sensor, Vin, iL1, iL2, Io, duty, PWM,
  K1, K2). Nama file mengikuti skenario: `ramp150_*`, `S1_vref_*`, `S2_*`, `S2mil_*`, `S3_vinnaik_*`,
  `S3_vinturun_*`, `U10_ovp_*`, `U10_short_out_*`, `K2_lepas_*`, `T4_ol_0654_*`, `AW_*`, `disarm_150V_*`.
- `hasil/ringkasan_transien.png`: start-up, S1, S2, S3 ketiga pengendali.

Angka penting (set hw, Vin 18 V, R 135 Ω):

| Besaran | H∞ | PI-FR | MPSO-PI |
|---|---|---|---|
| D tunak 150 V | 0,702 | 0,703 | 0,703 |
| iL1 tunak | 12,5 A | 12,5 A | 12,6 A |
| Masuk ±2 % dari start (ramp 300 V/s, 445 ms) | 447 ms | 1490 ms | 766 ms |
| S2 135 → 270 Ω, Vo puncak | 166,1 V | 170,9 V | 171,6 V |
| S3 Vin +10 %, Vo puncak | 162,4 V | 166,2 V | 166,8 V |
| S3 Vin −10 %, Vo minimum | 136,5 V | 133,6 V | 133,1 V |
| S2 versi MIL 135 → 101 Ω | trip OCP_IN_F | trip OCP_IN_S | trip OCP_IN_S |

Ini angka simulasi untuk memeriksa logika dan ambang, bukan pengganti MIL-2 atau pengukuran.
