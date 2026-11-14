# Skrip MATLAB — H∞ Controller QBC

Alur perancangan dan validasi **H∞ controller mixed-sensitivity** untuk Quadratic Boost
Converter, dari sintesis sampai Processor-in-the-Loop (PIL). Folder ini hanya berisi
pengendali H∞. Pengendali pembanding tidak disertakan.

Diperlukan MATLAB dengan **Robust Control Toolbox** (`augw`, `hinfsyn`), **Control System
Toolbox**, dan untuk uji PIL **Instrument Control** (`serialport`). Model Simulink (MIL-1, MIL-2,
SIL, PIL) dibangun dengan blok Simscape Electrical dan tidak disertakan di repositori ini.

## Urutan menjalankan

| Tahap | Folder | Skrip | Keluaran |
|---|---|---|---|
| 1. Sintesis (MIL-1) | `01_sintesis_MIL1/` | `hinf_synthesis_MIL1.m` | `Khinf_QBC_MIL1.mat`: K(s) orde 6, γ ≈ 0,61, bobot W₁–W₃, plot S/T/KS |
| | | `mil_metrics.m` | metrik respons MIL-1 (rise/settling time, overshoot, riak) |
| 2. Diskretisasi (MIL-2) | `02_diskretisasi_MIL2/` | `hinf_discretize_MIL2.m` | `Khinf_QBC_MIL2.mat`: K(z) Tustin, T<sub>s</sub> = 100 µs |
| | | `hinf_dloop_check.m` | margin & stabilitas loop diskret K(z)·P(z)·z⁻¹ |
| | | `mil_metrics2.m`, `mil_metrics2_vin.m` | metrik MIL-2: start-up, step beban, step V<sub>in</sub> ±10 % |
| 3. SIL | `03_SIL/` | `export_controller_c.m` | `controller_coeffs.h` dari `Khinf_QBC_MIL2.mat` |
| | | `gen_test_vectors.m` → `compare_via_mex.m` | uji unit `controller_step()` (MEX) terhadap `dlsim` |
| | | `mil_metrics_sil.m`, `mil_metrics_sil_vin.m` | metrik SIL (blok C Function) dibanding MIL-2 |
| 4. PIL | `04_PIL/` | `pil_stm32/pil_stm32.ino` | firmware PIL lockstep untuk STM32F407VGT (USB CDC) |
| | | `ujiC.m`, `ujiC2.m` | uji loopback dari MATLAB |
| | | `PILSerialController.m` | blok *MATLAB System* untuk model PIL Simulink |

## Parameter desain

V<sub>in</sub> = 18 V, V<sub>o</sub> = 150 V, R = 135 Ω, L₁ = 1 mH, L₂ = 3 mH,
C₁ = C₂ = 100 µF, f<sub>s</sub> = 10 kHz.
Bobot: W₁ (ω<sub>b</sub> = 100 rad/s, M₁ = 2, A₁ = 10⁻³), W₂ = 1/u<sub>max</sub>,
W₃ (ω<sub>bt</sub> = 1500 rad/s, M₃ = 5, ε₃ = 0,02).

## Catatan

- `controller_step.c/.h` dan `controller_coeffs.h` di `03_SIL/` dan `04_PIL/pil_stm32/` identik
  dengan yang dipakai firmware di `hil_stm32/` dan `keil_hal/App/`.
- Port serial default `COM5`. Sesuaikan dengan `serialportlist`.
- Simpan sketch PIL di folder bernama `pil_stm32` bersama ketiga file kontroler
  (aturan Arduino IDE).
