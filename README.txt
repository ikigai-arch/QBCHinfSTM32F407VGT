FIRMWARE KONTROLER H-INFINITY -- BOARD QBC
==========================================
Isi folder:
  hil_stm32.ino          sketch utama
  controller_step.c/.h   kontroler H-infinity (JANGAN diubah)
  controller_coeffs.h    koefisien orde 6, Ts = 100 us (JANGAN diubah)
  PINMAP.md              pin map

Mas Tino TIDAK perlu mengompilasi apa pun jika DevEBox sudah di-flash oleh
Leo. Cukup pasang DevEBox ke header PCB.

Power-up
  - Power DevEBox dari satu sumber saja (USB atau 5 V PCB).
  - Saat boot PWM MATI (duty 0). Kontrol baru jalan setelah perintah 'e'.

Kompilasi / flash ulang (hanya jika perlu)
  Arduino IDE + core STM32duino, board: Generic STM32F4 series -> Generic F407VGTx
  Tools -> USB support -> CDC (generic 'Serial' supersede U(S)ART)
  Upload method: STM32CubeProgrammer (DFU). Masuk DFU: BT0 ke 3V3, tekan reset.
  Setelah flash: lepas BT0, tekan reset.

Perintah serial (USB, 115200, akhiri newline)
  v<angka>  vref target (V), contoh v50      e  mulai kontrol
  d         stop + reset / hapus FAULT       p  status
  k<angka>  kalibrasi pengali Vo, contoh k0.98

Proteksi di firmware
  - Duty dijepit 0 .. 0,65
  - OVP latch di 165 V (3 sampel berturut-turut), atau pin FAULT aktif
  - vref maksimum 150 V, naik dengan ramp
  - Setelah FAULT: kirim 'd' untuk reset

Telemetri: T,ms,state,vref,vo,duty,cycles,overrun   (state 0=OFF 1=RUN 2=FAULT)
