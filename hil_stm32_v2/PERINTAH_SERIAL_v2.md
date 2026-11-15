# Perintah serial dan format keluaran firmware `hil_stm32_v2` (v0.9.1)

Port: USB CDC (micro-USB DevEBox). Baud tidak berpengaruh. Akhiri setiap perintah dengan Enter
(`\r`, `\n`, atau keduanya). Huruf besar/kecil sama saja. Pemisah argumen: spasi atau koma.

Terminal yang disarankan: PuTTY, CoolTerm, atau Serial Monitor Arduino dengan setelan **DTR aktif**.
Firmware memakai sinyal DTR sebagai tanda "ada yang mendengarkan" (lihat `link`).

## 0. Darurat

| Ketik | Efek |
|---|---|
| `!` | Langsung dieksekusi tanpa Enter. PWM mati pada periode berikutnya (≤ 100 µs setelah karakter diterima), lalu status FAULT (`USER`): K1 dibuka 20 ms kemudian, K2 setelah Vo < 30 V. |

Simpan `!` sebagai tombol cepat di terminal. Sakelar darurat fisik di PSU tetap wajib.

## 1. Status dan informasi

| Perintah | Keterangan |
|---|---|
| `help` atau `?` | daftar perintah singkat |
| `ver` | versi firmware |
| `st` | satu baris status: state, pengendali, vo, vin, iin, iout, T, d, vref (sekarang/target), K1, K2, kipas, sim, link, fault aktif, fault pertama, overrun, status capture |
| `cfg` | sysclk dan sumbernya (`HSE` atau `HSI!`), pengendali, anti-windup, batas duty, laju ramp, vref, koefisien kalibrasi keempat kanal, gain PI/MPSO-PI |
| `lim` | semua ambang proteksi yang berlaku sekarang |

## 2. Mesin status

```
            arm (Vin 5..22 V, Vo sisi beban < 5 V, NTC ok, T < tmax-5)
  OFF ──────────────────────► ARMING ── K1 tutup ── 300 ms ── K2 tutup ── 100 ms ──► ARMED
   ▲  │                          │ (Vin jatuh / Vo tidak naik → FAULT ARM)            │  │
   │  │ pwmtest                  │                                                ol │  │ cl
   │  ▼                          ▼                                                   ▼  ▼
   │ PWMTEST (relai dipaksa terbuka)                                        RUN_OL    RUN_CL
   │  │ stop                                                                   │ stop  │ stop
   │  └──► OFF                                                                 └► ARMED ◄┘
   │
   └── DISARM ◄── disarm / USB putus > 1 s      FAULT ◄── proteksi apa pun / '!'
         urutan buka: PWM mati → 20 ms → K1 buka → (Vo < 30 V atau Io < 0,1 A atau 5 s) → K2 buka
         DISARM selesai → OFF;  FAULT tetap FAULT sampai `clr` (fault latched)
```

| Perintah | Dari status | Efek |
|---|---|---|
| `arm` | OFF | mulai urutan relai (K1 dulu, lalu K2). PWM tetap mati. |
| `disarm` | apa pun selain OFF/FAULT | PWM mati, buka relai berurutan, lalu OFF |
| `stop` atau `x` | RUN_OL / RUN_CL → ARMED; PWMTEST → OFF; ARMING → DISARM | PWM mati, relai tetap |
| `clr` | FAULT | hapus fault **hanya setelah** urutan buka relai selesai (kalau belum: `TOLAK,relai_belum_selesai_dibuka`) → OFF |

Perintah yang mengubah status masuk antrean (16 slot) dan dieksekusi ISR kontrol pada tick berikutnya.
Balasan `OK` berarti perintah **diterima antrean**, bukan berarti sudah berhasil; hasilnya muncul
sebagai event `E,...,STATE,...` atau `E,...,TOLAK,...`.

## 3. Uji T1/T2: PWM tanpa daya

| Perintah | Keterangan |
|---|---|
| `pwmtest` | PWM aktif dengan K1/K2 **dipaksa terbuka**. Ditolak kalau V_IN > 2 V (`TOLAK,Vin_ada_saat_pwmtest...`), karena K1 yang lengket tidak bisa dideteksi dan C11/C13 akan terpompa tanpa beban. Kalau PSU dinyalakan di tengah pwmtest → FAULT `RELAY`. |
| `pwmtest force` | sama, tetapi V_IN boleh ada (misalnya catu logika diambil dari VDC). Pakai **hanya** setelah K1 dicek terbuka dengan multimeter (mode Ω, tanpa daya). |
| `d <D>` | duty target 0..0,85 di PWMTEST; duty naik/turun dengan laju `dr`. |
| `stop` | PWM mati → OFF |

## 4. Open-loop (T3 kalibrasi, T4)

| Perintah | Keterangan |
|---|---|
| `ol` | dari ARMED: PWM mulai dengan duty 0 |
| `d <D>` | duty target. Di RUN_OL dijepit ke `dmax` (default 0,66). Duty bergerak ke target dengan laju `dr`. |
| `dr <1/s>` | laju ramp duty, default 0,20 /s (0 → 0,654 dalam 3,3 s), maks 2 /s |
| `dmax <D>` | batas duty open-loop, maks 0,75 (batas keras). Contoh `dmax 0.70` untuk titik Tabel 4.10 dengan rugi. |
| `noload on/off` | default on: di RUN_OL, Vo > 40 V dengan Io < 20 mA selama 200 ms → FAULT `NOLOAD` |

## 5. Closed-loop (T5, T6)

| Perintah | Keterangan |
|---|---|
| `ctrl hinf` / `ctrl pi` / `ctrl mpso` | pilih pengendali. Hanya saat PWM mati (ditolak `PWM_masih_jalan`). State pengendali di-reset. Default setelah boot: `hinf`. |
| `aw on/off` | anti-windup H∞ (conditional integration di luar `controller_step.c`). Default on. `aw off` = replikasi MIL-2 murni. PI/MPSO-PI selalu memakai clamping bawaan blok Simulink. |
| `v <V>` | target Vref 0..160 V |
| `vr <V/s>` | laju ramp Vref, default 300 V/s. Maks 6000 V/s (= model MIL-2; arus start lebih besar, harness: 14,8 A). |
| `cl` | dari ARMED: mulai closed-loop. Vref mulai dari Vo terukur lalu naik dengan laju `vr` ke `v`. Ditolak kalau `v` belum diset. |
| `s <dV>` | step Vref seketika (S1), −20..+20 V. Pin TRIG PE6 HIGH 1 ms dan capture ikut terpicu. |
| `trig` | pulsa 1 ms di PE6 tanpa mengubah apa pun (untuk memicu osiloskop pada S2/S3) |

Duty closed-loop selalu dijepit 0,05..0,85 (sama dengan MIL-2/PIL).

## 6. Proteksi

Semua ambang berlaku di setiap mode yang memberi daya (K1, K2, atau PWM aktif). Penjelasan setiap
nilai ada di `qbc_config.h`.

| Kode fault | Kondisi (default) | Ambang bisa diubah dengan `lim` |
|---|---|---|
| `OVP` | Vo > 180 V selama 3 sampel (300 µs) | `lim ovp <V>` 5..185 |
| `ADC_SAT` | kode V_OUT atau I_IN ≥ 4090 (≥ 198 V atau ≥ 22 A, tidak terukur lagi) selama 2 sampel | – |
| `OCP_IN_F` | I_IN > 20 A selama 3 sampel | `lim iinf <A>` 0,2..21 |
| `OCP_IN_S` | rata-rata I_IN (τ 0,5 s) > 15,5 A | `lim iins <A>` 0,2..16 |
| `OCP_OUT_F` | I_OUT > 2,5 A selama 3 sampel | `lim ioutf <A>` 0,05..3 |
| `OCP_OUT_S` | rata-rata I_OUT (τ 0,5 s) > 1,8 A | `lim iouts <A>` 0,05..2 |
| `OV_IN` | Vin > 22 V | `lim vinmax <V>` 6..25 |
| `UV_IN` | Vin < 4 V saat PWM jalan (OL/CL) | – |
| `OTP` | suhu heatsink > 80 °C | `lim tmax <C>` 25..85 |
| `NTC` | NTC lepas/hubung singkat saat jalan | `ntc on/off` |
| `VO_PLAUS` | (a) Vo terukur turun > 40 V di bawah rata-rata 1 ms selama 3 sampel (K2 lepas / sensor putus), atau (b) Vo < 0,7·Vin − 3 V selama 5 ms | – |
| `DSAT` | duty closed-loop menempel 0,85: penghitung bocor (+2 jenuh, −1 tidak) mencapai 2 × 50 ms | `lim dsat <ms>` 1..200 |
| `NOLOAD` | lihat §4 | `noload on/off` |
| `ARM` | saat ARMING: Vin < 5 V setelah K1, atau Vo tidak naik setelah K2 | – |
| `RELAY` | di PWMTEST: Vo > 10 V atau I_IN > 0,3 A selama 5 sampel, atau Vin > 2 V tanpa `force` | – |
| `SIM_REAL` | di mode SIM: ADC asli melihat daya | – |
| `ISR_STALL` | ISR kontrol berhenti ≥ 3 ms (PWM dimatikan langsung dari loop) | – |
| `USER` | perintah `!` | – |

`lim` hanya bisa mengetatkan sampai batas keras (`*_HARD` di `qbc_config.h`). `limreset` kembali ke
default. Ambang tidak disimpan setelah reset. Untuk uji U10, turunkan ambang ke nilai yang aman
dicapai (misalnya `lim ovp 60` saat Vo 50 V), lalu naikkan Vo sedikit.

USB putus: kalau DTR hilang > 1 s saat ada daya, firmware melakukan DISARM (event
`LINK_USB_PUTUS->DISARM`). `link keep` mematikan kebijakan ini (tidak disarankan); `link stop` = default.

Kipas (PE0): `fan auto` (default: ON saat PWM jalan atau T > 40 °C, OFF di bawah 35 °C tanpa PWM),
`fan on`, `fan off`.

## 7. Kalibrasi T3

| Perintah | Keterangan |
|---|---|
| `raw <n>` | rata-rata dan simpangan baku kode ADC dari n sampel (1..65000; 1000 = 0,1 s). Hasil satu baris `R,...` |
| `cal <kanal> <a> <b>` | kanal `vo`/`vin`/`iin`/`iout`. Satuan teknik = a·kode + b. Hanya saat OFF. Ditolak kalau a menyimpang > 50 % dari nilai skematik (menangkap salah ketik dan skala v1). **Tidak tersimpan** setelah reset: salin ke `CAL_*` di `qbc_config.h`. |

Format `R`:
```
R,n=1000,vo=3101.52/s=1.20/x=150.0123,vin=2234.10/s=0.90/x=18.0011,iin=.../s=.../x=...,iout=.../s=.../x=...,temp=.../s=.../T=24.8
```
`vo=<rata-rata kode>/s=<simpangan baku kode>/x=<nilai teknik dengan kalibrasi saat ini>`.

Hubungan dengan panduan 4.3: `p = polyfit(N, x_ref, 1)` → `cal <kanal> p(1) p(2)`. Nilai `a` di
firmware adalah `p(1)` langsung (satuan per kode), bukan bentuk Pers. 48.

Nilai skematik (default): vo 0,048352 V/kode, vin 0,0080586 V/kode, iin 0,0053724 A/kode,
iout 0,0013431 A/kode, b = 0.

## 8. Logging

### Telemetri (default tiap 100 ms; `tel <ms>` 10..10000, `tel 0` = mati)

```
T,ms,state,ctrl,flags,vref,vo,vin,iin,iout,temp,duty,vo_min,vo_max,faults,isr_us,isr_us_max,ctrl_cyc,ovr,sat_ms
```
- vo, vin, iin, iout, duty: rata-rata semua sampel 100 µs di jendela itu; vo_min/vo_max: ekstrem di jendela.
- `flags` (bit): 0 SIM, 1 anti-windup, 2 K1, 3 K2, 4 kipas, 5 PWM aktif, 6 link USB, 7 NTC rusak.
- `isr_us`: lama ISR terakhir; `isr_us_max`: maksimum sejak boot; `ctrl_cyc`: siklus CPU pengendali
  terakhir (DWT) untuk Tabel siklus CPU; `ovr`: periode PWM yang terlewat ISR; `sat_ms`: total waktu duty
  CL jenuh atas.

### Event (selalu dikirim, prioritas di atas telemetri)

```
E,<ms>,STATE,<status>[,<fault>]       E,<ms>,FAULT,<kode>,<nilai saat trip>
E,<ms>,TOLAK,<alasan>                 E,<ms>,INFO,<apa>,<nilai>
```
INFO: `ARM_LANGKAH` (1 = K1, 2 = K2), `STEP` (Vref baru), `TRIG`, `CTRL`, `AW`, `SIM`,
`CAPTURE_SELESAI`, `K2_DIBUKA_C13_MASIH_BERMUATAN` (Vo saat K2 dibuka > 50 V: buang muatan dulu!),
`LINK_USB_PUTUS->DISARM`.

### Capture resolusi 100 µs (data cadangan T6, korelasi PIL)

| Perintah | Keterangan |
|---|---|
| `cap <pre%> <dec>` | siapkan buffer 4096 sampel, `pre` % sebelum pemicu (default 20), simpan tiap `dec` sampel (1 = 100 µs → jendela 409,6 ms; 5 → 2,05 s) |
| `capgo` | picu manual |
| `capdump` | kirim isi buffer |

Pemicu otomatis: `s` (src=1), `trig` (2), fault (3), `capgo` (4).

```
C,HDR,n=4096,pre=819,dt_us=100,src=1,ctrl=HINF,trig_ms=12345
C,i,t_ms,vo,vin,iout,duty
C,0,-81.9,150.02,18.001,1.1112,0.70213
...
C,END
```
t_ms = 0 di sampel pemicu. Capture menyimpan satuan teknik saat direkam.

## 9. Mode SIM (uji meja tanpa daya)

| Perintah | Keterangan |
|---|---|
| `sim on` / `sim off` | hanya dari OFF. Plant rata-rata QBC dihitung di chip; pin relai fisik **dipaksa LOW**, PWM keluar di PA8 seperti biasa |
| `sim r <ohm>` | beban model (5..1e5) |
| `sim vin <V>` | Vin model (0..30) |
| `sim par model` / `sim par hw` | set parasitik: `model` = MIL2.slx, `hw` = perkiraan hardware (qbc_hw_params.m) |

Kalau ADC asli melihat daya selama SIM (Vo > 10 V, I_IN > 1 A, atau I_OUT > 0,5 A) → FAULT `SIM_REAL`.
Gunanya: mencoba semua perintah (`arm`, `cl`, `s 5`, `ctrl`, `cap`) dan melihat PWM di osiloskop tanpa
risiko. Di mode SIM semua pemeriksaan (`arm`, proteksi, telemetri) memakai besaran model, termasuk
Vin = `sim vin`; ADC asli hanya dipakai untuk mendeteksi `SIM_REAL`.
