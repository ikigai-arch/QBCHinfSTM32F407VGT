# Review firmware board QBC v1 (`hil_stm32/` dan `keil_hal/`, branch `main`)

Tanggal: 15 Nov 2026. Dasar: repo `ikigai-arch/QBCHinfSTM32F407VGT` branch `main` (= salinan lokal
`STM32F407VGT/QBCHinfSTM32F407VGT-main/`), skematik KiCad `Quadratic_Boost_Converter_v0.1`, Proposal v10
sempro, Panduan Mengisi Bab 4, `qbc_hw_params.m`, `pi_params.h` (PIL_PI_MPSO).

Tingkat: **Bahaya hardware** (bisa merusak MOSFET/dioda/kapasitor/PSU atau membahayakan orang),
**Salah hasil uji** (data T1–T6 tidak valid), **Kualitas** (rawan masalah, perawatan).

Semua perbaikan ada di `hil_stm32_v2/`. File pengendali H∞ (`controller_step.c/.h`,
`controller_coeffs.h`) disalin tanpa perubahan (md5 sama dengan `pil_stm32` dan `MIL/Sil tahap akhir`:
`3db4f1b0…` / `ddd783c4…` / `0530604f…`).

## A. Masalah firmware v1

| # | Tingkat | Lokasi | Masalah | Perbaikan di v2 |
|---|---|---|---|---|
| 1 | Bahaya hardware | `hil_stm32.ino:35,108`; `keil_hal/Core/Src/main.c:43,135` | Satu-satunya proteksi selain OVP adalah pin FAULT PD8, padahal **PD8 tidak tersambung** di PCB (skematik `microcontroller`). Dengan pull-down, pin selalu LOW, jadi proteksi itu tidak pernah aktif. I_IN, I_OUT, V_IN, TEMP tidak dibaca sama sekali. | Proteksi lengkap di `qbc_core.c`: OVP, ADC jenuh, OCP masukan cepat/lambat, OCP keluaran cepat/lambat, OV/UV Vin, OTP, NTC rusak, plausibilitas Vo, DSAT, NOLOAD, RELAY, ISR macet. Setiap ambang beralasan di `qbc_config.h`. |
| 2 | Bahaya hardware | skematik `analog`/`high_voltage`; `hil_stm32.ino:99-122` | Sensor V_OUT terpasang **setelah K2**. Kalau K2 terbuka (atau kontaknya lepas), Vo terbaca 0, pengendali melihat galat 150 V dan menaikkan duty ke batas, sementara converter tanpa beban (DCM) → Vo naik tak terkendali tanpa terlihat sensor. | (a) Saat arm, K2 ditutup lalu Vo harus naik ke ≥ 0,7·Vin − 3 V (F_ARM). (b) Saat jalan, Vo terukur turun > 40 V di bawah rata-rata 1 ms selama 3 sampel → F_VO_PLAUS. (c) Cadangan: Vo < 0,7·Vin − 3 V selama 5 ms. Harness: K2 lepas di 150 V → Vo sebenarnya maks **154 V** (tanpa (b): 189 V). |
| 3 | Bahaya hardware | `hil_stm32.ino:212` | `if (Serial)` di STM32duino berisi `delay(10)`. Karena langkah kontrol dijalankan di `loop()`, kontrol **dan OVP berhenti 10 ms setiap 50 ms** (100 periode tanpa kontrol, duty terakhir tetap). `Serial.print` juga memblok selama DTR aktif tapi host tidak membaca. | Kontrol dan semua proteksi di ISR ADC (dipicu TIM1), tidak tergantung `loop()`. Keluaran teks lewat ring buffer non-blocking yang hanya mengirim sebanyak `availableForWrite()`. |
| 4 | Bahaya hardware | `hil_stm32.ino` (seluruh), `keil_hal/main.c` | Tidak ada watchdog. HardFault di Arduino membiarkan TIM1 terus mengeluarkan PWM dengan duty terakhir. (Keil: HardFault mematikan MOE, bagus, tapi tanpa watchdog.) | IWDG ~100 ms yang hanya di-reset bila ISR kontrol hidup; handler HardFault/MemManage/BusFault/UsageFault mematikan MOE dan membuka K1 lewat register; loop mematikan PWM bila ISR berhenti ≥ 3 ms. |
| 5 | Bahaya hardware | `hil_stm32.ino:143-148` | Perintah `d` langsung menghapus FAULT tanpa syarat, lalu `e` bisa menyalakan lagi seketika setelah OVP. | Fault latched. `clr` hanya diterima setelah urutan buka relai selesai; menyalakan lagi butuh `arm` → `ol`/`cl`. |
| 6 | Bahaya hardware | `keil_hal/Core/Src/main.c:106` | `adcReadOnce()` mengembalikan **0** bila `HAL_ADC_PollForConversion` timeout → Vo = 0 → pengendali menaikkan duty. | ADC injected dipicu hardware, tanpa polling. Lompatan Vo ke 0 tertangkap #2(b). |
| 7 | Bahaya hardware | `hil_stm32.ino` (tidak ada) | Tidak ada kebijakan saat USB terputus: PWM tetap jalan tanpa pengawasan. | DTR hilang > 1 s saat ada daya → DISARM (bisa dimatikan dengan `link keep`). |
| 8 | Salah hasil uji (pemblokir) | `hil_stm32.ino` (tidak ada) | v1 **tidak pernah menyalakan relai K1/K2** (pin PC1/PC0 tidak dikonfigurasi). Power stage tidak pernah mendapat daya, jadi T3–T6 tidak bisa dijalankan. | Mesin status OFF → ARMING (K1, 300 ms, K2, 100 ms) → ARMED → RUN; urutan buka PWM → 20 ms → K1 → (Vo < 30 V) → K2 supaya K2 tidak memutus 150 V DC. |
| 9 | Salah hasil uji | `hil_stm32.ino:53`; `keil_hal/main.c:51` | `VO_SCALE = 88,96` (placeholder 4×220k:10k). Skematik: R19/(R18+R37) = 10k/600k → 60. Vo terbaca **48 % terlalu tinggi**: pengendali menahan Vo sebenarnya di ~101 V saat Vref 150 V, dan OVP 165 V baru aktif di Vo sebenarnya ~111 V. | Skala keempat kanal dari skematik (`CAL_*`), perintah `raw`/`cal` untuk T3; `cal` menolak nilai yang menyimpang > 50 % dari skematik. |
| 10 | Salah hasil uji | `hil_stm32.ino:40,81`; `keil_hal/main.c:39,97` | Duty dijepit 0..0,65, padahal D tunak di titik nominal dengan rugi 0,68–0,70 (Tabel 4.17, panduan 4.7; harness set hw: 0,702). Vo 150 V @135 Ω tidak tercapai. H∞ tanpa anti-windup → state integrator terus naik selama jepitan aktif. MIL/PIL memakai 0,05–0,85. | Closed-loop 0,05–0,85 (sama dengan MIL-2/PIL). Open-loop dmax 0,66 default, maks 0,75. Anti-windup H∞ di luar file pengendali (`ctrl_mux.c`) + DSAT. Lihat B5. |
| 11 | Salah hasil uji | `hil_stm32.ino:42` | OVP 165 V trip palsu di S3: MIL-2 (Tabel 4.20) Vin +10 % memberi puncak +18,4 V (PI) dan +19,4 V (MPSO-PI) di atas 150 V. Harness set hw: 166,2 V (PI), 166,8 V (MPSO-PI); S2 135→270 Ω: 170,9 / 171,6 V. | OVP 180 V (alasan di `qbc_config.h`: di atas 172 V, di bawah skala penuh sensor 198 V dan rating C12 250 V). |
| 12 | Salah hasil uji | varian STM32duino "Generic F407VGTx" | Varian memakai **HSI** (±1 %). PWM T1 bisa meleset dari 10,00 kHz (kriteria panduan 4.1) dan USB di luar spesifikasi (±0,25 %). | `SystemClock_Config()` ditimpa: HSE 8 MHz → 168 MHz, USB 48 MHz; cadangan HSI dilaporkan di banner dan `cfg`. |
| 13 | Salah hasil uji | `hil_stm32.ino:87-91`; `keil_hal/main.c:103-116` | ADC dibaca dengan `analogRead()` (inisialisasi ulang ADC tiap panggilan) / polling di loop, pada saat sembarang di periode PWM. Menambah tunda yang tidak ada di model PIL dan menangkap riak 10 kHz secara acak (aliasing). | ADC1 injected dipicu TIM1 CH4 di **tengah interval ON** (atau tengah OFF bila D < 0,28), 4 kanal dalam 7,6 µs, ISR di JEOC, duty baru berlaku di update event berikutnya (= unit delay MIL-2/PIL). |
| 14 | Salah hasil uji | `hil_stm32.ino` | Hanya H∞. T6 butuh PI respons frekuensi dan MPSO-PI dengan ADC, PWM, proteksi identik, plus open-loop untuk T1/T4. | `ctrl hinf|pi|mpso` (gain dari `pi_params.h` hasil `gen_pi_params.m`, tidak dikarang), `pwmtest`, `ol`/`d`, `s`, pin TRIG PE6, capture 100 µs, `ctrl_cyc` (DWT) per pengendali. |
| 15 | Kualitas | `hil_stm32.ino:45` | Ramp Vref 6000 V/s (= model) di hardware menambah arus pengisian C2: harness iL1 puncak 14,8 A vs 12,7 A di 300 V/s. | Default 300 V/s; `vr 6000` tetap bisa untuk replikasi MIL. |
| 16 | Kualitas | `PINMAP.md` | Usang. Aktual (netlist KiCad, pad U6): V_IN = **PA4**, I_IN = **PA5**, I_OUT = **PA2**, TEMP = **PA7**, K1 = **PC1**, K2 = PC0, kipas = **PE0**; PD8 FAULT tidak tersambung. PE6 dan PE9 bebas. | `hw_f407.h` memakai pin aktual. PINMAP.md di `main` perlu diperbarui (belum diubah, karena `main` dipegang Mas Tino). |
| 17 | Kualitas | `hil_stm32.ino:74-77` | `onPeriod` + `tickFlag` di loop: jitter tergantung `loop()`, overrun hanya dihitung. | ISR prioritas 0, overrun dihitung dari UIF selama ISR dan dilaporkan di telemetri. |

## B. Temuan baru yang ditemukan harness PC pada v2 0.9.0 (sudah diperbaiki di 0.9.1)

| # | Tingkat | Lokasi (0.9.0) | Masalah | Perbaikan 0.9.1 |
|---|---|---|---|---|
| B1 | Salah hasil uji | `qbc_config.h` `IIN_SLOW_DEFAULT 14` | S3 Vin −10 %: iL1 tunak 14,0 A (set hw) → OCP lambat trip palsu. | 15,5 A (hard 16). |
| B2 | Salah hasil uji | `qbc_config.h` `IIN_FAST_DEFAULT 18` | S3 arah kembali (Vin 16,2 → 18 V, step ideal) memberi transien iL1 18,1 A → OCP cepat trip palsu. | 20 A (hard 21), **syarat Isat L1 ≥ 20 A** (keputusan K1). |
| B3 | Bahaya hardware | `qbc_core.c` DSAT | Penghitung DSAT berurutan direset setiap kali duty lepas dari 0,85. Anti-windup H∞ membuat duty berselang-seling jenuh/tidak, sehingga DSAT tidak pernah trip (harness: 837 ms jenuh ~50 %, berakhir OCP 19,8 A). | Penghitung bocor +2/−1. Harness: Vin 6 V @1350 Ω → F_DSAT. |
| B4 | Bahaya hardware | `qbc_core.c` plausibilitas Vo | Hanya cek rasio Vin selama 5 ms: saat K2 lepas, Vo sebenarnya naik ke 189 V sebelum trip. | Deteksi lompatan turun (A#2(b)): 154 V. |
| B5 | Bahaya hardware | `qbc_core.c` PWMTEST | K1 lengket + PSU menyala saat pwmtest: C11/C13 terpompa (69 V di D 0,5, terus naik di D tinggi), tidak terlihat sensor (V_OUT setelah K2) dan I_IN hanya sesaat. | `pwmtest` menolak Vin > 2 V dan trip bila PSU dinyalakan di tengah uji; `pwmtest force` untuk kasus catu logika dari VDC (risiko sisa didokumentasikan). |
| B6 | Salah hasil uji (mode SIM) | `qbc_sim.c` | Euler eksplisit dt = 10 µs tidak stabil untuk set `model` (redaman rC 10 mΩ): osilasi numerik ±5 V yang tidak ada di MIL; ketiga pengendali tampak "berosilasi" di mode SIM. | Euler semi-implisit; selisih dengan dt = 0,1 µs < 0,5 V (uji `plant_acc`). |

## C. Hasil uji tanpa board (ringkas; rinci di `test_pc/hasil/`)

- **Vektor H∞** (`MIL/Sil tahap akhir/e_test.csv`, `y_matlab_test.csv`): `controller_step` vs `dlsim`
  max |Δu| = 1,8e-7 (0,06 % dari max |u|); referensi double dari koefisien yang sama: 4e-12, jadi sisa
  galat murni float32. Pembungkus `ctrl_mux` (aw off) **bit-identik** dengan `controller_step` + jepit.
- **Presisi float32**: pada galat realistis selama 20 s open-loop, keluaran float32 menyimpang dari double
  maks 4,3e-4 duty (7 count PWM). Di closed-loop dikoreksi integrator; dicatat sebagai info.
- **Harness closed-loop** (30 uji, 193 cek, 0 gagal): boot aman, penolakan perintah, urutan relai,
  ramp 150 V untuk ketiga pengendali (set hw dan model), S1, S2 (135→270→135 Ω), S3 (dua arah),
  OVP/OCP/OTP/NTC/DSAT/NOLOAD/RELAY/SIM_REAL/USB putus/K2 lepas/darurat, perpindahan pengendali,
  capture, raw, telemetri. Invarian keselamatan (PWM mati di OFF/ARMED/FAULT, relai, batas duty, NaN)
  dicek di setiap tick.
- Prediksi titik nominal (set hw, H∞): D 0,702, iL1 12,5 A (panduan Tabel 4.17: 0,703 / 12,58 A).

## D. Catatan teknis yang perlu diketahui (bukan bug firmware)

| # | Catatan |
|---|---|
| T1 | Gain DC pengendali H∞ hanya **2,0** (integrator bocor, pole 0,99999). Galat tunak = D/2,0 ≈ **0,35 V di 150 V (0,23 %)**, sama di set model dan hw. Ini sifat `controller_coeffs.h` (juga ada di MIL-2/SIL/PIL), bukan bug v2. Cocokkan dengan ess H∞ di Tabel 4.19 sebelum menulis Bab 4. |
| T2 | **Anti-windup H∞ tidak bisa dibandingkan secara bermakna** di plant ini: step Vref ±20 V, beban dilepas, dan S1–S3 tidak membuat duty jenuh; skenario yang membuat jenuh (Vin turun jauh) berakhir OCP saat Vin pulih, dengan atau tanpa anti-windup. Pengaman yang benar-benar bekerja: DSAT 50 ms dan OCP_IN. `aw off` tersedia untuk replikasi MIL-2. |
| T3 | PI-FR (`pi_ctrl.c`) memakai `double` (soft-float di Cortex-M4: ~13 panggilan `__aeabi_d*`, perkiraan beberapa µs). Sama dengan PIL; cek `ctrl_cyc` di telemetri. |
| T4 | Open-loop D 0,654 @18 V/135 Ω memberi **~121 V** dengan rugi (harness set hw), bukan 150 V. Tabel 4.10 memang membandingkan dengan model berugi. |
| T5 | Skenario S2 berbeda antara MIL-2 (Gambar 4.10: beban **naik** 135 → 101 Ω) dan panduan hardware 4.9 (135 → **270** → 135 Ω). Dengan rugi set hw, 101 Ω (223 W) butuh iL1 ~17–20 A → OCP trip di ketiga pengendali. |

## E. Belum bisa dipastikan tanpa board

1. Kompilasi v0.9.1 dengan arduino-cli + STM32duino (0.9.0 sudah: 61,5 kB / 45 kB). Perubahan 0.9.1
   hanya di file portabel; semua file kecuali `.ino` sudah dikompilasi untuk Cortex-M4 (clang 18,
   header CMSIS/HAL resmi ST) tanpa peringatan.
2. Timing nyata: lama ISR (`isr_us_max`), posisi cuplik ADC terhadap PWM (PE9 vs PA8), overrun.
3. Perilaku USB CDC STM32duino (DTR, `availableForWrite`) dan kebijakan link putus.
4. Polaritas dan waktu relai (D882 + RAYEX-L90S), arus inrush saat K1 menutup.
5. Watchdog (IWDG) dan handler fault: hanya bisa diuji di board.
6. NTC: jenis tidak tertulis di skematik (diasumsikan 10 k/B3950).
7. Semua angka harness memakai model rata-rata tanpa riak, tanpa dead-time gate driver, dan tanpa
   filter Sallen-Key; pembacaan arus di board akan berbeda.

## F. Keputusan yang perlu ditanyakan

| # | Ke | Pertanyaan |
|---|---|---|
| K1 | Mas Tino | Arus saturasi L1 (dan L2)? Firmware butuh Isat L1 ≥ 20 A agar OCP cepat 20 A aman. Kalau lebih kecil, OCP diturunkan dan S3 dibuat dengan slew PSU (harness: ramp 10 ms → 17,2 A). Rating daya shunt R23 (15,5² × 5 mΩ = 1,2 W)? |
| K2 | Mas Tino | Simbol DevEBox: nomor pin ganda PA2/PA14 (pin 76) dan PA6/PA13 (pin 72). Pastikan I_OUT benar di PA2 (bukan PA14 = SWCLK). |
| K3 | Mas Tino | Arus LED TLP250 ~3,4 mA (R24 470 Ω) di bawah I_FLH maks 5 mA → gate driver bisa tidak menyala pasti. Ganti R24 ~220 Ω? |
| K4 | Mas Tino | D7 (D2 model, STTH8S06D 8 A) dialiri ~8,8 A rata-rata di titik nominal; C13 tanpa bleeder. |
| K5 | Mas Tino | Jumper Supply Select / konektor External Supply: apakah catu logika bisa dipasang tanpa VDC (untuk T1/T2 dengan `pwmtest` tanpa `force`)? Jenis dan posisi NTC? |
| K6 | Dosen | S2 hardware memakai 135 → 270 → 135 Ω (panduan) sedangkan MIL-2 135 → 101 Ω. Jika S2 hardware harus sama dengan MIL, 101 Ω melebihi batas arus dengan rugi (T5). Pilih yang mana untuk Bab 4? |
| K7 | Dosen | Galat tunak H∞ 0,23 % (T1): diterima sebagai sifat desain, atau perlu integrator murni? (Mengubah pengendali berarti mengulang MIL/SIL/PIL; firmware tidak mengubahnya.) |
