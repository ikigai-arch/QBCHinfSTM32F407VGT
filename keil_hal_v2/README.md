# Firmware board QBC v2 — versi Keil uVision (STM32 HAL)

Versi Keil dari `hil_stm32_v2/` (Arduino). **Logika kontrol, proteksi, relai, perintah serial,
dan telemetri sama persis**: semua file di `App/` adalah salinan identik dari `hil_stm32_v2/`.
Bedanya hanya cara berkomunikasi dengan PC:

| | Arduino (`hil_stm32_v2/`) | Keil (folder ini) |
|---|---|---|
| Build | Arduino IDE + STM32duino | Keil uVision (ARM Compiler 6) atau `make` (arm-none-eabi-gcc) |
| Serial | USB CDC (micro-USB DevEBox) | **USART3: PB10 = TX, PB11 = RX**, 115200 8N1, lewat modul USB-TTL **3,3 V** |
| Deteksi PC/kabel putus | sinyal DTR (otomatis) | **heartbeat**: perintah `hb <ms>` (lihat di bawah) |

> **Status (15 Nov 2026):** semua file `.c` (App, Core, HAL) sudah dikompilasi untuk Cortex-M4
> (clang 18, header CMSIS/HAL di folder `Drivers/`) **tanpa peringatan**, dan sudah di-link dengan
> lld tanpa simbol ganda (sekitar 46 kB flash dan 43 kB RAM, belum termasuk pustaka C).
> **Project Keil belum pernah dibuka di Keil dan belum di-build dengan ARM Compiler 6**, dan
> firmware belum pernah jalan di board. Kalau Keil memberi error, kirim teks error-nya.

Folder `keil_hal/` (tanpa `_v2`) adalah firmware lama v1. Folder itu **tidak aman untuk uji
daya** (duty dijepit 0,65, relai tidak pernah dinyalakan, skala Vo salah, OVP 165 V); lihat
`hil_stm32_v2/REVIEW_firmware_v1.md`.

## 1. Build dan flash di Keil

1. Pasang pack **Keil::STM32F4xx_DFP** lewat Pack Installer. Kalau Keil minta update versi pack
   saat membuka project, setujui.
2. Buka `MDK-ARM/qbc_v2.uvprojx`.
3. Cek *Options for Target*:
   - **Target**: ARM Compiler = *Use default compiler version 6*, Floating Point Hardware =
     *Single Precision*, *Use MicroLIB* dicentang (sudah).
   - **C/C++ (AC6)**: Define `USE_HAL_DRIVER,STM32F407xx` (sudah). Include path sudah berisi
     `../App` dan `../Core/Inc`. Language C99.
   - **Asm**: kalau startup error, pilih *Assembler Option: armasm (Arm Syntax)*.
   - **Debug**: ST-Link Debugger, Port SW. **Settings → Flash Download**: algoritma
     *STM32F4xx 1MB Flash*, centang *Reset and Run*.
4. **Build (F7)**. Grup `App/QBC_v2` berisi `qbc_core.c`, `ctrl_mux.c`, `qbc_sim.c`, `hw_f407.c`,
   `controller_step.c`, `pi_ctrl.c`. Harus 0 error. Kalau ada peringatan dari file `App/`, kirim.
5. **Download (F8)**. Hasil `qbc_v2.axf` dan `qbc_v2.bin` ada di `MDK-ARM/qbc_v2/`.

Sambungan ST-Link ke DevEBox (header SWD): SWDIO, SWCLK, GND, 3V3.

Alternatif tanpa Keil: `make` di folder ini (arm-none-eabi-gcc + newlib-nano) menghasilkan
`build/qbc_v2.hex` dan `.bin`, lalu flash dengan STM32CubeProgrammer.

> **Debugger dan watchdog.** Firmware menyalakan watchdog (IWDG, ~100 ms). Watchdog berhenti saat
> CPU di-*halt* debugger, tetapi saat breakpoint aktif, **PWM TIM1 tetap jalan dengan duty terakhir**.
> Jangan memasang breakpoint saat power stage diberi daya.

## 2. Sambungan serial

| DevEBox | Modul USB-TTL (CP2102 / CH340 / FT232, **level 3,3 V**) |
|---|---|
| PB10 (TX) | RX |
| PB11 (RX) | TX |
| GND | GND (GND MCU, **bukan** GND_MOS) |

Jangan sambungkan pin 5 V/3V3 modul ke board kalau DevEBox sudah mendapat catu dari PCB.
Terminal: 115200, 8N1, tanpa flow control, akhir baris CR atau CRLF.

## 3. Heartbeat (pengganti DTR)

USART tidak punya sinyal DTR, jadi firmware tidak tahu kalau PC mati atau kabel lepas. Perintah
tambahan khusus versi ini:

| Perintah | Efek |
|---|---|
| `hb` | tampilkan setelan heartbeat |
| `hb 0` | **default saat boot**: link dianggap selalu tersambung (sama dengan `link keep`) |
| `hb <ms>` (200..10000) | link dianggap putus bila tidak ada satu byte pun diterima selama `<ms>`; 1 s kemudian firmware melakukan DISARM (PWM mati, relai dibuka berurutan). `arm` juga ditolak selama link putus. |

**Untuk semua uji dengan daya, pakai `tools/qbc_term.py`.** Skrip ini mengirim `hb 1000` saat
tersambung dan heartbeat setiap 200 ms. Skrip juga menyimpan log lengkap, telemetri CSV, dan
capture CSV.

```
pip install pyserial
python tools/qbc_term.py COM5
```

Kalau memakai PuTTY atau terminal biasa, biarkan `hb 0`. Saat itu **tidak ada** perlindungan
terhadap PC hang atau kabel lepas, jadi jaga tombol darurat PSU.

## 4. Perintah, telemetri, dan urutan uji

Sama dengan versi Arduino (folder `hil_stm32_v2/`):

- `PERINTAH_SERIAL_v2.md`: semua perintah, kode fault, format telemetri, event, capture.
- `UJI_PERTAMA_v2.md`: urutan menyalakan board dari nol sampai siap T4. Penyesuaian untuk Keil:
  - Tahap 0.2: ganti "Verify di Arduino IDE" dengan langkah §1 di atas.
  - Tahap 1.1: banner berakhir dengan `/ keil_hal_v2 (USART3 115200)`.
  - Tahap 1.3: flags telemetri 66 tetap benar (bit 6 = link; dengan `hb 0` selalu 1).
  - Tahap 1.11 dan 3.12 (uji USB putus): jalankan `qbc_term.py`, lalu tutup skripnya atau cabut
    kabel USB-TTL. Harus muncul `LINK_USB_PUTUS->DISARM` sekitar 2 s kemudian (heartbeat 1 s +
    1 s). Tanpa heartbeat (`hb 0`) uji ini tidak berlaku.

Pin firmware (dari netlist KiCad): PWM PA8, V_OUT PA3, I_IN PA5, V_IN PA4, I_OUT PA2, TEMP PA7,
K1 PC1, K2 PC0, kipas PE0, TRIG PE6, pin ISR PE9. PD8 dan PD9 tidak dipakai.

## 5. Isi folder

```
keil_hal_v2/
├─ MDK-ARM/qbc_v2.uvprojx        project Keil (buka ini)
├─ MDK-ARM/startup_stm32f407xx.s startup (sintaks ARM)
├─ App/                          SALINAN IDENTIK dari hil_stm32_v2/ (jangan diedit di sini saja)
│   ├─ qbc_core.c/.h             mesin status, proteksi, perintah, telemetri
│   ├─ qbc_config.h              semua ambang beserta alasannya
│   ├─ ctrl_mux.c/.h             pemilih H∞ / PI / MPSO-PI + anti-windup
│   ├─ hw_f407.c/.h              TIM1, ADC injected, ISR, clock, watchdog, handler fault
│   ├─ qbc_sim.c/.h              model converter untuk mode SIM
│   └─ controller_*, pi_*        pengendali (TIDAK diubah, sama dengan MIL/SIL/PIL)
├─ Core/Src/main.c               loop serial + heartbeat (khusus versi ini)
├─ Core/Src/app_uart.c           USART3 ber-buffer interrupt (tidak pernah memblokir)
├─ Core/Src/stm32f4xx_it.c       SysTick, USART3 (ADC dan fault handler ada di hw_f407.c)
├─ Core/Src/stm32f4xx_hal_msp.c  pin USART3
├─ Drivers/                      STM32F4 HAL + CMSIS
├─ Makefile, gcc/                build GCC opsional
└─ tools/qbc_term.py             terminal + heartbeat + logger
```

**Menjaga kedua versi tetap sama.** Kalau mengubah ambang atau logika, ubah di
`hil_stm32_v2/`, jalankan `make test` di `hil_stm32_v2/test_pc/`, lalu salin file yang sama ke
`keil_hal_v2/App/`. Cek dari folder repo:

```
for f in keil_hal_v2/App/*; do cmp -s "$f" "hil_stm32_v2/$(basename "$f")" || echo "BEDA: $f"; done
```
