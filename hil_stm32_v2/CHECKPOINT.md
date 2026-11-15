# CHECKPOINT sesi Fable — 15 Nov 2026 (lanjutan, selesai)

## Status
- Firmware v2 **0.9.1** di folder ini. Semua butir "Belum dikerjakan" dari checkpoint 19:40 sudah dikerjakan:
  harness PC + vektor H∞ (`test_pc/`), REVIEW_firmware_v1.md, PERINTAH_SERIAL_v2.md, UJI_PERTAMA_v2.md,
  CHANGELOG.md.
- Harness: 30 uji, 193 cek, 0 gagal. Vektor H∞ vs dlsim: max |Δu| 1,8e-7 (float32), wrapper bit-identik.
- Harness menemukan 6 masalah di 0.9.0 (REVIEW bagian B), semuanya diperbaiki di 0.9.1.

## Kompilasi
- 0.9.0: arduino-cli + STM32duino 3.1.0 berhasil (61,5 kB / 45 kB).
- 0.9.1: BELUM dikompilasi dengan arduino-cli (jaringan sesi memblok downloads.arduino.cc).
  Semua file .c dikompilasi untuk Cortex-M4 dengan clang 18 + header CMSIS/HAL resmi ST: 0 peringatan.
  Langkah 0.2 di UJI_PERTAMA_v2.md wajib sebelum flash.

## Menunggu keputusan
K1–K7 di REVIEW_firmware_v1.md bagian F (Isat L1, pin ganda DevEBox, TLP250, D7, Supply Select/NTC,
skenario S2 101 vs 270 Ω, galat tunak H∞ 0,23 %).
