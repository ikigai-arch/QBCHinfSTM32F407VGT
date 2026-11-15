# CHANGELOG firmware board QBC

## v2 0.9.1 (15 Nov 2026), hasil harness PC

Perubahan setelah 0.9.0 (yang sudah dikompilasi dengan arduino-cli + STM32duino 3.1.0). Semua karena
temuan harness `test_pc/` (rincian REVIEW_firmware_v1.md bagian B).

- OCP masukan lambat 14 → **15,5 A** (hard 16): trip palsu di S3 Vin −10 % (iL1 tunak 14,0 A).
- OCP masukan cepat 18 → **20 A** (hard 21): trip palsu saat Vin kembali 16,2 → 18 V (18,1 A).
  Syarat Isat L1 ≥ 20 A (belum diverifikasi).
- DSAT memakai penghitung bocor (+2 jenuh / −1 tidak); sebelumnya tidak pernah trip bila duty
  berselang-seling jenuh akibat anti-windup.
- Plausibilitas Vo: tambah deteksi lompatan turun > 40 V terhadap rata-rata 1 ms (3 sampel). K2 lepas
  di 150 V: Vo sebenarnya maks 154 V (sebelumnya 189 V).
- `pwmtest` menolak V_IN > 2 V dan trip `RELAY` bila PSU dinyalakan di tengah uji; `pwmtest force`
  baru. Ambang arus RELAY 1 A/10 sampel → 0,3 A/5 sampel.
- `qbc_sim.c`: Euler semi-implisit (stabil di dt = 10 µs untuk set model).
- Belum berubah dari 0.9.0 dan belum dikompilasi ulang dengan arduino-cli: (dari checkpoint) SIM 10
  sub-langkah, suhu dihitung tiap 10 ms, `SAMPLE_T_MAX_US` 60. Semua file kecuali `.ino` sudah
  dikompilasi untuk Cortex-M4 dengan clang 18 + header CMSIS/HAL resmi tanpa peringatan.
- Komentar `qbc_config.h`: S2 hardware = 135 → 270 → 135 Ω (panduan 4.9), bukan 101 Ω.

## v2 0.9.0 (15 Nov 2026), perbedaan terhadap v1 (`hil_stm32/`, branch `main`)

**Arsitektur**
- Kontrol, proteksi, relai, dan ramp pindah dari `loop()` ke ISR ADC1 injected yang dipicu TIM1
  (10 kHz). `loop()` hanya mengurus teks USB (non-blocking). Logika inti di `qbc_core.c` (C portabel,
  bisa diuji di PC); lapisan register di `hw_f407.c`.
- ADC dicuplik di tengah interval ON (tengah OFF bila D < 0,28), 4 kanal dalam satu urutan injected;
  duty baru berlaku di periode berikutnya (= unit delay MIL-2/PIL).
- Clock HSE 8 MHz → 168 MHz (varian Generic F407 memakai HSI).
- Watchdog IWDG ~100 ms, handler fault inti mematikan PWM dan K1, deteksi ISR macet dari loop.

**Pin (dari netlist KiCad, bukan PINMAP.md)**
- V_IN PA4, I_IN PA5, I_OUT PA2, TEMP PA7, V_OUT PA3, K1 PC1, K2 PC0, kipas PE0, PWM PA8,
  TRIG PE6, pin ISR PE9. PD8 (FAULT) tidak dipakai karena tidak tersambung.

**Fungsi baru**
- Mesin status OFF / ARMING / ARMED / RUN_OL / RUN_CL / PWMTEST / DISARM / FAULT; default aman saat boot.
- Relai K1/K2 berurutan saat arm dan saat buka (K2 dibuka setelah Vo < 30 V).
- Tiga pengendali dalam satu firmware: H∞ (file asli), PI respons frekuensi, MPSO-PI (gain dari
  `pi_params.h` yang dibuat `gen_pi_params.m`). Pilihan hanya saat PWM mati.
- Duty closed-loop 0,05–0,85 (sebelumnya 0–0,65); anti-windup H∞ di luar file pengendali.
- Open-loop dengan ramp duty dan `dmax`; `pwmtest` untuk T1/T2.
- Proteksi lengkap (lihat PERINTAH_SERIAL_v2.md §6), ambang bisa diketatkan untuk U10.
- Kalibrasi T3: `raw`, `cal`; skala default dari skematik (V_OUT ×60, bukan 88,96).
- OVP 165 → 180 V (S2/S3 PI dan MPSO-PI mencapai 166–172 V).
- Ramp Vref default 300 V/s (sebelumnya 6000 V/s).
- Pin TRIG PE6 untuk S1, capture 4096 sampel @100 µs, telemetri 20 kolom dengan siklus CPU pengendali.
- Mode SIM: plant rata-rata di chip untuk uji meja tanpa daya.

**Tidak berubah**
- `controller_step.c/.h`, `controller_coeffs.h` (md5 sama dengan `pil_stm32`), `pi_ctrl.c/.h`,
  `pi_params.h` (sama dengan PIL_PI_MPSO).
- `hil_stm32/` dan `keil_hal/` v1 dibiarkan utuh sebagai pembanding.
