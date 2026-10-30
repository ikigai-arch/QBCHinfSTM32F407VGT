# Firmware H-infinity QBC — versi Keil uVision (STM32 HAL)

Versi cadangan dari `hil_stm32/hil_stm32.ino` untuk Keil uVision (MDK-ARM).
Pin, parameter, proteksi, dan perintah serial **sama** dengan versi Arduino.
`App/controller_step.c`, `controller_step.h`, `controller_coeffs.h` adalah file
yang sama persis dengan MIL-2 / SIL / PIL (tidak diubah).

> **Status:** berhasil dikompilasi dengan arm-none-eabi-gcc 13.3 terhadap
> STM32F4 HAL V1.8.5. **Belum diuji di board dan belum dibuka di Keil.**
> Setelah flash, cek dulu di osiloskop sebelum disambung ke power stage
> (lihat bagian *Uji pertama*).

Kalau DevEBox sudah di-flash versi Arduino dan berjalan, versi ini **tidak
perlu** dipakai. Flash versi ini akan menimpa firmware Arduino di board.

## Beda dengan versi Arduino

| | Arduino (`hil_stm32/`) | Keil HAL (folder ini) |
|---|---|---|
| Serial perintah/telemetri | USB CDC (micro-USB DevEBox) | **USART3: PB10 = TX, PB11 = RX**, 115200 8N1, lewat modul USB-TTL 3,3 V |
| Clock | diatur core STM32duino | 168 MHz dari HSE 8 MHz (otomatis pindah ke HSI 16 MHz jika HSE gagal) |
| ADC sampling time | default core | 84 siklus (≈4 µs) |
| Akhir baris telemetri | `\r\n` | `\r\n` |

Lainnya sama: PWM TIM1_CH1 di PA8 10 kHz, ADC1_IN3 di PA3, fault PD8 (aktif
HIGH, pull-down), LED PD9, duty 0..0,65, vref maks 150 V dengan ramp, OVP
165 V (3 sampel), PWM mati saat boot.

## Isi folder

```
keil_hal/
├─ MDK-ARM/qbc_hinf.uvprojx      project Keil (buka ini)
├─ MDK-ARM/startup_stm32f407xx.s startup (sintaks ARM)
├─ Core/Src/main.c               logika utama, init TIM1/ADC/GPIO/clock
├─ Core/Src/app_uart.c           USART3 ber-buffer (tidak memblokir loop kontrol)
├─ Core/Src/stm32f4xx_it.c       handler interrupt
├─ Core/Src/stm32f4xx_hal_msp.c  init pin ADC & UART
├─ Core/Inc/stm32f4xx_hal_conf.h konfigurasi HAL (HSE = 8 MHz)
├─ App/controller_*              kontroler H-infinity (JANGAN diubah)
├─ Drivers/                      STM32F4 HAL V1.8.5 + CMSIS (dari GitHub ST/ARM)
├─ Makefile, gcc/                build GCC opsional
```

## Build & flash di Keil

1. Pasang pack **Keil::STM32F4xx_DFP** (Pack Installer). Kalau Keil minta
   update versi pack saat membuka project, setujui saja.
2. Buka `MDK-ARM/qbc_hinf.uvprojx`.
3. Cek *Options for Target*:
   - **Target**: ARM Compiler = *Use default compiler version 6*; Floating Point
     Hardware = *Single Precision*; *Use MicroLIB* boleh dicentang.
   - **C/C++**: Define `USE_HAL_DRIVER,STM32F407xx` (sudah terisi).
   - **Asm**: kalau startup error, pilih *Assembler Option: armasm (Arm Syntax)*.
   - **Debug**: ST-Link Debugger, Port SW. **Settings → Flash Download**:
     algoritma *STM32F4xx 1MB Flash*, centang *Reset and Run*.
4. Build (F7), lalu Download (F8).
5. File `qbc_hinf.hex` dan `qbc_hinf.bin` muncul di `MDK-ARM/qbc_hinf/`.

Sambungan ST-Link ke DevEBox (header SWD): SWDIO, SWCLK, GND, 3V3.

Tanpa ST-Link: flash `qbc_hinf.bin` lewat STM32CubeProgrammer mode USB/DFU
(BT0 ke 3V3, reset, alamat 0x08000000, lalu lepas BT0 dan reset).

### Jika project tidak mau terbuka / error konfigurasi

File `.uvprojx` ditulis manual, bukan dari CubeMX. Kalau bermasalah, buat
project baru dari STM32CubeMX (STM32F407VGTx, Toolchain *MDK-ARM*,
HSE kristal 8 MHz, SYSCLK 168 MHz, APB1 /4, APB2 /2), **jangan** aktifkan
periferal apa pun, lalu:
- ganti `main.c`, `stm32f4xx_it.c`, `stm32f4xx_hal_msp.c` hasil CubeMX dengan
  file di `Core/Src/` folder ini, dan tambahkan `Core/Src/app_uart.c`,
  `Core/Inc/app_uart.h`, `Core/Inc/main.h`;
- tambahkan `App/controller_step.c` dan include path `App`;
- pastikan modul HAL ADC, TIM, UART aktif di `stm32f4xx_hal_conf.h`.

## Build dengan GCC (opsional)

```
make            # butuh arm-none-eabi-gcc di PATH
```
Hasil di `build/qbc_hinf.{elf,hex,bin}`.

## Uji pertama (wajib, sebelum power stage)

Sambungkan USB-TTL: TX modul → **PB11**, RX modul → **PB10**, GND → GND.
Buka terminal serial 115200, akhiran *newline*.

1. Setelah reset muncul: `QBC H-inf HAL firmware siap. PWM OFF...`
2. Kirim `p` → cek `sysclk=168000000 (HSE)` dan `arr=16799`.
   Kalau tertulis `(HSI)`, kristal HSE tidak start; firmware tetap jalan di 168 MHz.
3. Osiloskop di PA8: harus **datar 0 V** (PWM OFF).
4. Kirim `v50`, `m0`, `e` → PA8 jadi pulsa **10 kHz**, duty naik sampai 0,65.
   Kolom `duty` di telemetri harus cocok dengan lebar pulsa di layar.
5. Kirim `d` → PA8 kembali datar 0 V. Kirim `a` untuk kembali ke ADC asli.
6. Kolom `overrun` harus tetap 0; `cycles` = siklus CPU satu langkah kontroler.

## Perintah serial

| Perintah | Fungsi |
|---|---|
| `e` | enable, mulai kontrol (ramp vref dari Vo terukur); hanya dari state OFF |
| `d` | disable, PWM mati, reset kontroler, hapus FAULT |
| `v<angka>` | vref target (V), 0..150, contoh `v50` |
| `k<angka>` | kalibrasi pengali Vo, 0,1..10, contoh `k0.98` |
| `m<angka>` | paksa Vo = angka (V), abaikan ADC — uji meja, loop terbuka |
| `a` | kembali baca Vo dari ADC |
| `p` | status (termasuk sysclk dan ARR timer) |

Telemetri tiap 50 ms: `T,ms,state,vref,vo,duty,cycles,overrun,manual`
(state 0 = OFF, 1 = RUN, 2 = FAULT).

## Lisensi pihak ketiga

`Drivers/STM32F4xx_HAL_Driver` dan `Drivers/CMSIS/Device` © STMicroelectronics
(BSD-3-Clause), `Drivers/CMSIS/Include` © Arm (Apache-2.0). File lisensinya
ada di masing-masing folder.
