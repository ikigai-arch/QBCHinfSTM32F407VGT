/* qbc_config.h -- konstanta board QBC v2 (STM32F407VGT DevEBox + PCB KiCad v0.1)
 *
 * SEMUA ambang proteksi di sini punya alasan tertulis. Nilai yang belum bisa
 * dipastikan dari datasheet/skematik ditandai "TODO verifikasi".
 *
 * Rujukan singkat:
 *   [SCH]  Rancangan Desain Hardware/Quadratic_Boost_Converter_v0.1 (= repo GitHub hardware/kicad)
 *   [PROP] Proposal v10 sempro (Tabel 3.3, 3.9, 4.9, 4.17, 4.19, 4.20)
 *   [PAND] Panduan_Mengisi_Bab4 (15 Nov 2026)
 *   [MOD]  model rata-rata rugi qbc_avg_deriv8.m, set 'hw' (qbc_hw_params.m)
 *
 * Batas KERAS (*_HARD_*) tidak bisa dilampaui lewat perintah serial. Perintah
 * "lim" hanya boleh mengetatkan (menurunkan ambang atas / menaikkan ambang
 * bawah) untuk uji proteksi U10.
 */
#ifndef QBC_CONFIG_H
#define QBC_CONFIG_H

#define QBC_FW_VERSION   "hil_stm32_v2 0.9.1 (2026-11-15)"

/* ---------------- Waktu ---------------- */
#define QBC_FSW_HZ        10000.0f      /* = 1/CTRL_TS kontroler (Ts = 100 us)            */
#define QBC_TS            1.0e-4f
#define QBC_TICKS_PER_MS  10u

/* ---------------- Rantai sensing (default dari [SCH] lembar analog) ----------------
 * x = a*kode + b. Ganti dengan hasil kalibrasi T3 (Tabel 4.9) lewat perintah "cal".
 *  V_OUT : R19/(R18+R37) = 10k/600k = 1/60         -> a = 3.3/4095*60   [V/kode]
 *  V_IN  : R7/R5 = 10k/100k = 1/10                  -> a = 3.3/4095*10
 *  I_IN  : R23 5 mOhm x (R6/R2 = 30k/1k = 30) = 0,15 V/A -> a = 3.3/4095/0.15
 *  I_OUT : R29 20 mOhm x (R16/R14 = 30)       = 0,60 V/A -> a = 3.3/4095/0.60
 * Skala penuh ADC (3,3 V): Vo 198 V, Vin 33 V, Iin 22 A, Iout 5,5 A.
 * Catatan: v1 memakai VO_SCALE = 88,96 (placeholder) -> salah 48 %. */
#define CAL_VO_A    (3.3f/4095.0f*60.0f)
#define CAL_VIN_A   (3.3f/4095.0f*10.0f)
#define CAL_IIN_A   (3.3f/4095.0f/0.15f)
#define CAL_IOUT_A  (3.3f/4095.0f/0.60f)
#define CAL_VO_B    0.0f
#define CAL_VIN_B   0.0f
#define CAL_IIN_B   0.0f
#define CAL_IOUT_B  0.0f

/* NTC: J1 (pin2 = +3,3 V PCB) -> NTC -> node -> R11 10k ke GND, lalu R12 3,9k + C7 100 nF ke PA7.
 * Jenis NTC tidak tertulis di skematik. TODO verifikasi: diasumsikan 10 kOhm @25 C, B = 3950. */
#define NTC_R0_OHM      10000.0f
#define NTC_T0_K        298.15f
#define NTC_BETA        3950.0f
#define NTC_RPULL_OHM   10000.0f        /* R11 */
#define NTC_OPEN_CODE   20u             /* < ~ -40 C: NTC lepas/putus                     */
#define NTC_SHORT_CODE  4070u           /* NTC hubung singkat                              */

/* ---------------- Duty ----------------
 * Closed-loop: 0,05..0,85 IDENTIK dengan MIL-2 empat pengendali dan PIL (pi_params.h D_MIN/D_MAX,
 * [PROP] Subbab 3.4.4). Alasan batas atas tidak diturunkan:
 *  - D tunak dengan rugi 0,68-0,70 di titik nominal ([PAND] 4.7, Tabel 4.17); [MOD] set hw:
 *    150 V @135 Ohm butuh D = 0,70; Vin -10 % (S3) butuh ~0,72 (harness test_pc: 0,722).
 *  - Bobot W2 H-inf memakai u_max = 0,2 (0,654 -> 0,85) sebagai ruang transien.
 *  - Puncak gain model rugi ada di D = 0,79-0,84 (IL1 ~ 40 A). Daerah itu dijaga oleh OCP I_IN
 *    (IL1 nominal 12,6 A) dan fault DSAT (duty menempel di DMAX > DSAT_MS), bukan dengan memotong
 *    duty, supaya transien sama dengan MIL/PIL.
 * Open-loop (T4): default maksimum 0,66 (>= 0,654 untuk Tabel 4.10). [MOD] Vo @135 Ohm:
 *   set model 0,66 -> ~145 V, set hw ~125 V. Dapat dinaikkan sampai OL_DMAX_HARD = 0,75
 *   (set hw: Vo ~180 V @135 Ohm = di ambang OVP). */
#define DUTY_CL_MIN      0.05f
#define DUTY_CL_MAX      0.85f
#define OL_DMAX_DEFAULT  0.66f
#define OL_DMAX_HARD     0.75f
#define PWMTEST_DMAX     0.85f          /* T1/T2: relai terbuka, tidak ada daya            */
#define OL_DRATE_DEFAULT 0.20f          /* 1/s: 0 -> 0,654 dalam 3,3 s ([PAND] 4.4: naik bertahap) */
#define OL_DRATE_HARD    2.0f

/* ---------------- Referensi ---------------- */
#define VREF_MAX_HARD    160.0f         /* S1 = 150 -> 155 V; 160 V memberi sisa 20 V ke OVP */
#define VREF_RATE_DEFAULT 300.0f        /* V/s: 0 -> 150 V dalam 0,5 s. Model memakai 6000 V/s
                                           (25 ms); pada hardware itu menambah C2*Vo*dV/dt ~ 90 W
                                           -> arus masukan +5 A saat start, mendekati OCP.          */
#define VREF_RATE_HARD   6000.0f        /* boleh disamakan dengan model (perintah "vr 6000")       */

/* ---------------- Proteksi ---------------- */
/* OVP Vo. MIL-2 (Tabel 4.20): Vin +10 % memberi +13,3 V (H-inf), +18,4 V (PI), +19,4 V (MPSO-PI)
 * di atas 150 V -> puncak 169,4 V; S1 bekerja di 155 V. v1 memakai 165 V -> trip palsu di S3 PI.
 * 180 V: di atas 169,4 V + toleransi, masih di bawah skala penuh sensor 198 V (pembacaan valid),
 * jauh di bawah C12 snubber 250 V, C11/C13 450 V, D6/D7/D9 600 V, Q1 650 V. 3 sampel = 300 us. */
#define OVP_V_DEFAULT    180.0f
#define OVP_V_HARD       185.0f
#define OVP_N            3u
/* OCP arus masukan (= iL1, filter Sallen-Key 3,1 kHz -> nilai ~rata-rata).
 * Nominal dengan rugi 12,4-12,6 A ([PAND] Tabel 4.17 / [MOD]); harness test_pc (set hw): 12,5 A tunak,
 * S3 Vin -10 % 14,0 A tunak dengan puncak transien 17,5 A (H-inf), S2 135->270 Ohm 12,6 A.
 * Cepat 20 A / 3 sampel: menangkap hubung singkat dan daerah puncak gain (IL1 ~ 40 A). Harness: Vin
 * KEMBALI 16,2 -> 18 V (S3 arah kedua, step ideal) memberi transien 18,1 A -> v0.9.0 (18 A) trip palsu.
 * 20 A = 18,1 A + 10 %, masih di bawah skala penuh 22 A (ADC_SAT di 4090 kode = 22,0 A).
 * SYARAT: Isat L1 >= 20 A (TODO verifikasi dengan Mas Tino). Jika lebih kecil, turunkan ke Isat dan
 * buat Vin S3 dengan slew PSU (harness: ramp 10 ms menurunkan transien, lihat uji vin_step).
 * Lambat 15,5 A rata-rata (tau 0,5 s): di atas 14,0 A (S3) + 10 %. v0.9.0 memakai 14 A -> trip palsu S3
 * (ditemukan harness). Shunt R23: 15,5^2 x 5 mOhm = 1,2 W.
 * TODO verifikasi: arus saturasi L1, rating daya shunt R23, dan D7 (STTH8S06D 8 A rata-rata;
 * ID2 ~ D*IL1 = 8,8 A di nominal SUDAH di atas rating, [PAND] 3.2). */
#define IIN_FAST_DEFAULT 20.0f
#define IIN_FAST_HARD    21.0f
#define IIN_FAST_N       3u
#define IIN_SLOW_DEFAULT 15.5f
#define IIN_SLOW_HARD    16.0f
#define ISLOW_TAU_S      0.5f
/* OCP arus keluaran. Nominal 1,11 A; S2 hardware [PAND] 4.9: 135 -> 270 -> 135 Ohm (Io turun ke 0,56 A).
 * Catatan: MIL-2 [PROP] Gambar 4.10 memakai beban NAIK 135 -> 101 Ohm (1,49 A, 223 W). Dengan rugi set hw
 * itu membutuhkan iL1 ~17 A tunak -> OCP_IN (14 A lambat / 18 A cepat) trip; lihat test_pc uji s2_mil.
 * Skala penuh 5,5 A.
 * Cepat 2,5 A (hubung singkat keluaran); lambat 1,8 A (~270 W, 1,6x daya desain 167 W). */
#define IOUT_FAST_DEFAULT 2.5f
#define IOUT_FAST_HARD    3.0f
#define IOUT_FAST_N       3u
#define IOUT_SLOW_DEFAULT 1.8f
#define IOUT_SLOW_HARD    2.0f
/* Vin. Desain 18 V, S3 16,2-19,8 V; uji tegangan rendah T4 mulai 9-12 V ([PAND] 4.4).
 * Maks 22 V: di atas 19,8 V + 10 %; pada OL D 0,66 Vin 22 V memberi Vo ideal 190 V (OVP menangkap).
 * Skala penuh sensor 33 V. Min 5 V untuk arm (uji tegangan rendah), 4 V saat jalan (sumber jatuh). */
#define VIN_MIN_ARM      5.0f
#define VIN_UV_RUN       4.0f
#define VIN_MAX_DEFAULT  22.0f
#define VIN_MAX_HARD     25.0f
/* Suhu heatsink (NTC). Q1 45N65M5 Tj maks 150 C, elko 105 C di dekatnya -> heatsink 80 C trip.
 * TODO verifikasi: jenis & posisi NTC, Rth heatsink. Kipas PE0 ON > 40 C atau saat PWM jalan. */
#define TEMP_TRIP_DEFAULT 80.0f
#define TEMP_TRIP_HARD    85.0f
#define TEMP_FAN_ON       40.0f
#define TEMP_FAN_OFF      35.0f
/* Duty menempel di DUTY_CL_MAX > 50 ms di closed-loop = Vo tidak tercapai (beban lebih, Vin jatuh,
 * sensor/K2 bermasalah, melewati puncak gain). H-inf MIL tidak pernah saturasi selama itu di S1-S3.
 * Penghitung "bocor" (+2 saat jenuh, -1 saat tidak): anti-windup H-inf membuat duty berselang-seling
 * jenuh/tidak (harness: 837 ms jenuh ~50 %) sehingga penghitung berurutan v0.9.0 tidak pernah trip.
 * Trip bila fraksi jenuh > 1/3 cukup lama; jenuh terus-menerus tetap trip setelah 50 ms. */
#define DSAT_MS_DEFAULT   50u
/* Plausibilitas sensor Vo (sensor V_OUT ada SETELAH K2 [SCH]): dengan K1 & K2 tertutup, Vo >= Vin
 * dikurangi 3 jatuh dioda pada setiap D. Jika Vo < 0,7*Vin - 3 V selama 5 ms -> sensor/K2 rusak. */
#define VOPLAUS_RATIO     0.7f
#define VOPLAUS_OFFS_V    3.0f
#define VOPLAUS_MS        5u
/* Tanpa beban di open-loop ([PROP] 3.5: jangan OL tanpa beban; DCM -> Vo naik tak terkendali).
 * TODO verifikasi setelah T3: offset kanal I_OUT harus << 20 mA. */
#define NOLOAD_IOUT_A     0.020f
#define NOLOAD_VO_V       40.0f
#define NOLOAD_MS         200u
/* Mode SIM (uji meja): kalau ADC asli melihat daya, SIM dihentikan. */
#define SIM_REAL_VO_MAX   10.0f
#define SIM_REAL_VIN_MAX  40.0f         /* Vin boleh ada (K1 terbuka), batas cuma sanity */
/* PWMTEST (T1/T2): [PAND] 4.1 "hanya catu logika". Jika sumber daya ada di VDC dan K1 lengket, PWM
 * memompa C11/C13 tanpa beban (sensor V_OUT setelah K2 tidak melihatnya; harness: 69 V dalam 3 s di
 * D 0,5 dan terus naik). Maka pwmtest default menolak V_IN > 2 V. "pwmtest force" mengizinkan Vin ada
 * (mis. catu logika diambil dari VDC lewat Supply Select) dengan deteksi I_IN yang lebih ketat. */
#define PWMTEST_VIN_MAX   2.0f
#define PWMTEST_IIN_A     0.3f
#define PWMTEST_IIN_N     5u
/* Lepasnya K2 saat jalan: sensor V_OUT (setelah K2) jatuh ke 0 dalam satu sampel, padahal C2 100 uF
 * tidak bisa turun secepat itu (beban nominal: 11 V/ms). Jika Vo terukur turun > 40 V di bawah
 * rata-rata 1 ms selama 3 sampel -> F_VO_PLAUS. Pemeriksaan rasio Vin (5 ms) tetap ada sebagai cadangan.
 * Harness: tanpa ini Vo SEBENARNYA naik ke ~189 V sebelum trip. */
#define VODROP_V          40.0f
#define VODROP_N          3u
#define VODROP_TAU_S      0.001f
/* Link USB (DTR) hilang > 1 s saat PWM jalan -> PWM berhenti (policy "link stop", default). */
#define LINK_LOSS_MS      1000u

/* ---------------- Relai (RAYEX-L90S, NO; aktif HIGH lewat D882, pull-down basis 5,1k) -------- */
#define K1_SETTLE_MS      300u          /* tutup K1 -> C11/C13 terisi ~Vin lewat L1/L2 (<1 ms) + pantul */
#define K2_SETTLE_MS      100u
#define K1_OPEN_DELAY_MS  20u           /* PWM mati -> arus induktor habis -> buka K1                  */
#define K2_OPEN_VO_V      30.0f         /* buka K2 setelah C13 dibuang ke beban (hindari busur DC 150 V) */
#define K2_OPEN_IO_A      0.10f
#define K2_OPEN_TMO_MS    5000u
#define ARM_VO_EXT_MAX    5.0f          /* ada tegangan di sisi beban sebelum arm (mis. baterai) -> tolak */

/* ---------------- Titik cuplik ADC (Subbab 3.3: tengah interval ON atau OFF) ---------------- */
#define SAMPLE_MIDON_ABOVE  0.32f       /* D >= 0,32 -> tengah ON  (t = D*T/2)          */
#define SAMPLE_MIDOFF_BELOW 0.28f       /* D <= 0,28 -> tengah OFF (t = (1+D)*T/2)      */
#define SAMPLE_T_MIN_US     5.0f
#define SAMPLE_T_MAX_US     60.0f       /* sisa >= 40 us untuk konversi 4 kanal (7,6 us) + ISR */

/* ---------------- Logging ---------------- */
#define CAP_N             4096u         /* sampel per capture (8 byte/sampel = 32 kB)  */
#define TEL_MS_DEFAULT    100u

#endif /* QBC_CONFIG_H */
