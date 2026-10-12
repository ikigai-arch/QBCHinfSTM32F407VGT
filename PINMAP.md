# Pin map -- STM32F407VGT (DevEBox) <-> PCB QBC

Status: USULAN, menunggu persetujuan Mas Tino. Setelah disetujui = dibekukan.
Tegangan logika 3,3 V. Fsw = 10 kHz, kontroler Ts = 100 us.

| Fungsi            | Pin STM32 | Peripheral        | Arah  | Catatan                                              |
|-------------------|-----------|-------------------|-------|------------------------------------------------------|
| PWM -> TLP250     | PA8       | TIM1_CH1          | out   | Pull-down di PCB agar tidak floating saat reset/flash |
| Vout (sensing)    | PA3       | ADC1_IN3          | in    | Dari diff amp + LPF analog. Maks 3,3 V. Proteksi RC + clamp |
| VC1 (cadangan)    | PA2       | ADC1_IN2          | in    | Belum dipakai firmware                               |
| Vin (cadangan)    | PA4       | ADC1_IN4          | in    | Belum dipakai firmware                               |
| Arus (cadangan)   | PA5       | ADC1_IN5          | in    | Belum dipakai firmware                               |
| Fault dari proteksi | PD8     | GPIO in (pull-down) | in  | Level aktif = HIGH (konfirmasi dengan desain proteksi) |
| LED status        | PD9       | GPIO out          | out   | Opsional                                             |
| Power             | 5V/3V3, GND | -               | -     | Satu sumber saja: USB ATAU 5 V PCB, jangan keduanya  |

Jangan dipakai di PCB: PA11/PA12 (USB), PA13/PA14 (SWD), PA9/PA10 (UART1),
serta pin yang terhubung ke tombol/LED onboard DevEBox.
GND analog (sensing) dipisahkan dari jalur PWM/gate-driver, bertemu di satu titik.
