# Pin map -- STM32F407VGT (DevEBox) <-> PCB QBC

Tegangan logika 3,3 V. Fsw = 10 kHz, kontroler Ts = 100 us.
PCB mengikuti tabel ini (bukan sebaliknya).

## Dipakai firmware

| Fungsi              | Pin STM32 | Peripheral          | Arah | Catatan |
|---------------------|-----------|---------------------|------|---------|
| PWM -> DRIVE_MOSFET | **PA8**   | TIM1_CH1            | out  | Pull-down di PCB. Konfirmasi: aktif HIGH? (duty 0 harus = MOSFET OFF) |
| V_OUT (sensing)     | **PA3**   | ADC1_IN3            | in   | Dari diff amp + LPF analog. Maks 3,3 V, proteksi RC + clamp |
| Fault dari proteksi | **PD8**   | GPIO in (pull-down) | in   | Aktif HIGH -- konfirmasi dengan desain proteksi |
| LED status          | PD9       | GPIO out            | out  | Opsional |

## Khusus versi Keil (`keil_hal/`)

| Fungsi            | Pin STM32 | Peripheral | Catatan |
|-------------------|-----------|------------|---------|
| Serial TX         | PB10      | USART3_TX  | ke RX modul USB-TTL 3,3 V |
| Serial RX         | PB11      | USART3_RX  | ke TX modul USB-TTL 3,3 V |

Versi Arduino (`hil_stm32/`) memakai USB CDC di port micro-USB, jadi PB10/PB11
hanya perlu bisa dijangkau (header/pin test) jika versi Keil yang dipasang.
PB10/PB11 jangan dipakai untuk fungsi lain di PCB.

## Ada di PCB, belum dipakai firmware (pin dibiarkan seperti skema Mas Tino)

| Fungsi       | Pin STM32 | Peripheral | Catatan |
|--------------|-----------|------------|---------|
| V_IN         | PA5       | ADC1_IN5   | |
| I_IN         | PA6       | ADC1_IN6   | |
| I_OUT        | PA7       | ADC1_IN7   | |
| TEMP1        | PB0       | ADC1_IN8   | |
| Relay_Input  | PC2       | GPIO out   | Perlu pull-down/up agar OFF saat boot |
| Relay_Output | PC0       | GPIO out   | Perlu pull-down/up agar OFF saat boot |
| Fan          | PE3       | GPIO out   | |

## Jangan dipakai
PA11/PA12 (USB), PA13/PA14 (SWD), PA9/PA10 (UART1), PA3 selain untuk V_OUT.

## Perubahan yang diminta dari skema awal
- DRIVE_MOSFET: PE6 (TIM9_CH2) -> **PA8 (TIM1_CH1)**
- V_OUT: PA4 -> **PA3**
- Tambah input fault di **PD8**

## Power
Satu sumber saja: USB ATAU 5 V PCB, jangan keduanya. GND analog dipisah dari
jalur PWM/gate-driver, bertemu di satu titik.
