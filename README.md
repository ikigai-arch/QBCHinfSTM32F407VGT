# QBC H-infinity — firmware STM32F407VGT (DevEBox)

Kontroler H-infinity diskret (orde 6, Ts = 100 µs, PWM 10 kHz) untuk
quadratic boost converter. Kode kontroler (`controller_step.c/.h`,
`controller_coeffs.h`) sama persis dengan yang dipakai di MIL-2, SIL, dan PIL.

| Folder | Isi |
|---|---|
| `hil_stm32/` | Versi utama: Arduino IDE + STM32duino. Serial lewat USB (micro-USB). **Sudah diuji di board (PWM di osiloskop).** |
| `keil_hal/`  | Versi cadangan: Keil uVision / STM32 HAL. Serial lewat USART3 (PB10/PB11). Terkompilasi, belum diuji di board. |
| `PINMAP.md`  | Pin map DevEBox ↔ PCB QBC. |

Pin, parameter, proteksi, dan perintah serial kedua versi sama. Pilih salah
satu; flash yang satu akan menimpa yang lain.
