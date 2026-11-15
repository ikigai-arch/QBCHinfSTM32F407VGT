# Uji pertama board QBC dengan firmware `hil_stm32_v2` (v0.9.1)

Urutan menyalakan board dari nol sampai siap T4 (open-loop daya penuh), lalu catatan singkat untuk
closed-loop pertama. Selaras dengan *Panduan Mengisi Bab 4* (T1–T6, U10) dan checklist 6.2 di sana.

**Aturan umum (berlaku di setiap tahap)**

- Satu tahap belum lolos = jangan lanjut ke tahap berikutnya. Catat semua di buku log (tanggal, Vin,
  D/Vref, R, setelan probe, suhu ruang).
- Terminal serial selalu terbuka dengan DTR aktif dan tombol cepat `!`. Simpan log terminal ke file.
- PSU daya: batas arus selalu aktif. Mulai dari nilai di tabel tiap tahap, naikkan hanya setelah lolos.
- Sebelum menyentuh board: `disarm`, PSU OFF, tunggu event `STATE,OFF`, lalu ukur Vo dan tegangan C1
  dengan multimeter. C11/C13 (100 µF/450 V) **tidak punya bleeder** (temuan review #16): pasang resistor
  bleeder di terminal keluaran sejak tahap 3.
- Jangan memasang ground probe di GND MCU dan GND_MOS bersamaan (panduan 2.4).
- Firmware ini **belum pernah dijalankan di board**. Hasil harness PC (`test_pc/`) hanya membuktikan
  logika; timing ISR, ADC, relai, dan USB baru terbukti di tahap 1–3.

Notasi: `> perintah` = ketik di terminal. `E,...` dan `T,...` = keluaran firmware.

---

## Tahap 0. Persiapan di PC (tanpa board)

| # | Langkah | Kriteria lolos |
|---|---|---|
| 0.1 | (Opsional, disarankan) Jalankan harness: buka MSYS2/WSL di `hil_stm32_v2/test_pc`, `make test`. | `TOTAL: 30 uji, 193 cek, 0 gagal` dan `HASIL: SEMUA LOLOS` di uji vektor H∞. |
| 0.2 | Arduino IDE: Board "Generic STM32F4 series", Board part number "Generic F407VGTx", USB support "CDC (generic 'Serial' supersede U(S)ART)", Optimize "Smallest (-Os default)", Upload method "STM32CubeProgrammer (SWD)" atau DFU. Buka `hil_stm32_v2.ino`, **Verify**. | Kompilasi tanpa error. Ukuran sekitar 62 kB flash / 45 kB RAM (versi 0.9.0 terkompilasi 61,5 kB / 45 kB dengan core 3.1.0). Peringatan dari file core boleh ada; peringatan dari file `qbc_*`, `ctrl_mux`, `hw_f407` harus nol. |
| 0.3 | Pastikan folder sketch hanya berisi file v2 (subfolder `test_pc` diabaikan IDE). | – |

> v0.9.1 belum dikompilasi dengan arduino-cli (jaringan sesi tidak bisa mengunduh core). File portabel
> dan `hw_f407.c` sudah dikompilasi untuk Cortex-M4 dengan clang + header CMSIS/HAL resmi tanpa
> peringatan, tapi langkah 0.2 tetap wajib. Kalau ada error, kirim pesan error-nya.

## Tahap 1. DevEBox saja (lepas dari PCB, atau PCB tanpa catu apa pun)

Tujuan: firmware boot aman, clock benar, ISR jalan, mode SIM bekerja. Tidak ada daya sama sekali.

| # | Perintah / tindakan | Yang dilihat | Kriteria lolos |
|---|---|---|---|
| 1.1 | Flash, buka terminal. | Banner `# hil_stm32_v2 0.9.1 ...`, `# reset: POWER-ON` (atau `PIN-NRST`), `# clock: HSE 168 MHz`, `# PWM OFF, K1/K2 terbuka.` | Clock **HSE**. Kalau `HSI (HSE GAGAL)`, cek kristal 8 MHz DevEBox, jangan lanjut (T1 butuh 10,00 kHz tepat). |
| 1.2 | `> cfg` lalu `> lim` | sysclk=168000000(HSE), dcl=0.05..0.85, dmax_ol=0.660, vr=300, CAL sesuai §7 PERINTAH_SERIAL, GAIN PI dan MPSO sama dengan `pi_params.h`; `LIM ovp=180.0 iinf=20.00 iins=15.50 ...` | Semua angka sama dengan dokumen. |
| 1.3 | Biarkan telemetri jalan 1 menit. | `T,...,OFF,HINF,66,...` (flags 66 = anti-windup + link USB), `isr_us` (catat; perkiraan < 30 µs), `ovr` = 0 dan tidak naik. | `ovr` tetap 0; `isr_us_max` < 40 µs. |
| 1.4 | Osiloskop CH1 di PA8, CH2 di PE9 (pin ISR). | PA8 LOW terus. PE9 pulsa tiap 100,0 µs. | PA8 tidak pernah HIGH. Periode PE9 = 100,0 µs (10,00 kHz). |
| 1.5 | Multimeter di PC1 (K1), PC0 (K2), PE0 (kipas). | 0 V | Semua LOW. |
| 1.6 | `> sim par hw`, `> sim on`, `> arm` | `E,...,INFO,SIM,1.00`, `STATE,ARMING`, `ARM_LANGKAH,1.00`, 300 ms kemudian `ARM_LANGKAH,2.00`, `STATE,ARMED`. | PC1/PC0 **tetap 0 V** (relai fisik dipaksa terbuka di SIM). |
| 1.7 | `> v 150`, `> cl` | `STATE,RUN_CL`; telemetri vo naik ±0,5 s ke ~149,7 V, duty ~0,70, iin ~12,5 A (angka model). Osiloskop: PA8 PWM 10 kHz, duty naik ke ~70 %. CH2 PE9: pulsa ISR jatuh **di tengah interval ON** PA8. | Vo model stabil di 149,6–149,8 V (H∞ punya galat tunak ~0,35 V, lihat REVIEW #T5). `ovr` = 0. `isr_us_max` < 40 µs. |
| 1.8 | `> cap 20 1`, `> s 5`, tunggu `CAPTURE_SELESAI`, `> capdump` | CH3 osiloskop di PE6: pulsa 1 ms saat `s 5`. Dump 4096 baris `C,...`. | Pulsa PE6 1,0 ms. Dump lengkap sampai `C,END`. |
| 1.9 | `> ctrl pi` (harus ditolak), `> stop`, `> ctrl pi`, `> cl`; ulangi untuk `mpso`, lalu `hinf`. | `TOLAK,PWM_masih_jalan` untuk yang pertama; ketiga pengendali mencapai ~150 V. Catat `ctrl_cyc` tiap pengendali. | Semua mencapai 150 ± 0,5 V tanpa fault. PI butuh ~1,5 s. |
| 1.10 | `> !` | `STATE,FAULT,USER`; PA8 langsung LOW. `> clr` → `STATE,OFF`. | PA8 LOW dalam 1 periode. |
| 1.11 | Tutup terminal (DTR hilang) saat RUN_CL di SIM, buka lagi. | `INFO,LINK_USB_PUTUS->DISARM`, lalu `STATE,OFF`. | DISARM ±1 s setelah terminal ditutup. |
| 1.12 | `> sim off` | `INFO,SIM,0.00` | – |

## Tahap 2. PCB, hanya catu logika (T1, T2)

PSU daya di VDC **dicabut**. Catu logika dari konektor External Supply (cek posisi jumper Supply Select
dengan Mas Tino). Panduan 4.1: "hanya catu logika".

| # | Perintah / tindakan | Yang dilihat | Kriteria lolos |
|---|---|---|---|
| 2.1 | Nyalakan catu logika, buka terminal, `> st` | vin ≈ 0, vo ≈ 0, iin dan iout dekat 0, T = suhu ruang ± 3 °C. | Tidak ada fault. Kalau `T=nan`, NTC tidak terbaca (cek J1). |
| 2.2 | `> raw 10000` | baris `R,...` | Simpan sebagai **offset nol** keempat kanal (titik pertama T3). `s` (noise) < 3 kode. |
| 2.3 | `> fan on`, `> fan off`, `> fan auto` | kipas berputar/berhenti | Kipas mengikuti perintah. |
| 2.4 | **T1**: `> pwmtest`, `> d 0.3` (tunggu 1,5 s), lalu `0.5`, `0.654`. Probe ×10 di TP13/PA8, ground GND MCU. | 10,00 kHz, +Width = D × 100 µs. | Sesuai panduan 4.1: f = 10,00 kHz, selisih duty < 1 poin persen. Isi Tabel 4.2. |
| 2.5 | **T2**: tetap `pwmtest`. Probe di TP8 (gate), ground GND_MOS/TP7 (lepas probe CH1 dari GND MCU dulu). | Amplitudo 14–15 V, rise/fall. | Panduan 4.2. Isi Tabel 4.3. **Kalau amplitudo kurang atau tepi lambat > 4 µs**, ini terkait arus LED TLP250 ~3,4 mA (REVIEW #15): hentikan dan tanyakan Mas Tino sebelum tahap 3. |
| 2.6 | `> stop`. Uji `pwmtest` menolak Vin: tidak bisa diuji di tahap ini (VDC kosong). | – | – |

## Tahap 3. Power stage tegangan rendah (Vin 6–9 V)

Beban 135 Ω (resistor daya) + bleeder di terminal keluaran. PSU daya 6 V, batas arus 2 A. Probe
tegangan di Vo terminal (diferensial atau ×10 dengan ground di GND daya). Probe arus di kabel +Vin.

| # | Perintah / tindakan | Yang dilihat | Kriteria lolos |
|---|---|---|---|
| 3.1 | PSU ON 6 V. `> st` | vin ≈ 6,0 V (bandingkan multimeter di terminal), vo ≈ 0 (sensor V_OUT ada **setelah K2**, jadi 0 sebelum arm). | V_IN terukur dalam ±3 % dari multimeter (skala skematik). |
| 3.2 | `> pwmtest` | `TOLAK,Vin_ada_saat_pwmtest...` | Ditolak (K1 lengket tidak bisa dideteksi). |
| 3.3 | `> arm` | Klik K1, 300 ms kemudian klik K2. `STATE,ARMED`. Telemetri vo ≈ Vin − 3 jatuh dioda (harness: 15,6 V di 18 V → ~4 V di 6 V). Probe arus: lonjakan saat K1 menutup (harness di 18 V: 4,8 A). | Urutan K1 → K2 terdengar dan terlihat di flags (70 = K1, lalu 78 = K1+K2). Tidak ada `FAULT,ARM`. |
| 3.4 | `> disarm` | `STATE,DISARM`, K1 buka, K2 buka setelah Vo < 30 V, `STATE,OFF`. | Tidak ada `K2_DIBUKA_C13_MASIH_BERMUATAN`. |
| 3.5 | `> arm`, `> ol`, `> d 0.3` | Duty naik pelan (0,2 /s). Vo naik. | Bandingkan dengan Tabel 4.1 (LTspice 12 V/D 0,3 → 22–24,5 V; di 6 V kira-kira separuhnya). Vo telemetri vs multimeter dalam ±3 %. |
| 3.6 | `> raw 10000` di D 0,3 (dan nanti D 0,5) | `R,...` | Titik kalibrasi pertama V_OUT dan I_IN (catat multimeter/probe arus bersamaan). |
| 3.7 | `> d 0.5` | Vo naik (ideal 6/0,25 = 24 V, dengan rugi lebih rendah). iin di telemetri ≈ probe arus Mean. | I_IN telemetri vs probe arus dalam ±5 % (skala skematik, sebelum kalibrasi). |
| 3.8 | **Uji proteksi U10 (OVP)**: di D 0,5, `> lim ovp <Vo sekarang + 3>`, lalu `> d 0.55`. Osiloskop: CH1 PA8, CH2 koil K1 (atau PC1), trigger CH2 turun. | `FAULT,OVP,...`; PA8 LOW; K1 buka 20 ms kemudian; K2 buka setelah Vo < 30 V. | PWM mati ≤ 400 µs setelah Vo melewati ambang (3 sampel + 1). Catat waktu respons. `> limreset`, `> clr`. |
| 3.9 | **U10 (OCP masukan)**: `> arm`, `> ol`, `> d 0.5`, `> lim iinf <iin sekarang × 0,8>` | `FAULT,OCP_IN_F` dalam < 1 ms. | Trip + urutan relai benar. `> limreset`, `> clr`. |
| 3.10 | **U10 (OCP keluaran)** dengan `lim ioutf`, dan **suhu** dengan `lim tmax <T sekarang + 2>` lalu panaskan NTC dengan jari/hair dryer. | `FAULT,OCP_OUT_F`, `FAULT,OTP` | Trip. `> limreset`, `> clr`. |
| 3.11 | **Darurat**: saat OL jalan, ketik `!` | PWM mati segera, relai buka berurutan. | – |
| 3.12 | **USB putus**: saat OL jalan, cabut kabel USB. | PWM mati ~1 s kemudian, relai buka. (Board tetap hidup dari catu logika.) | DISARM terjadi. Colok lagi: `st` menunjukkan OFF, fault kosong. |
| 3.13 | **Tanpa beban**: lepas resistor beban (bleeder tetap), `> arm`, `> ol`, `> d 0.3`. | Vo naik; `FAULT,NOLOAD` setelah Vo > 40 V selama 200 ms (atau OVP). | Trip sebelum Vo > 60 V. Kalau bleeder menarik > 20 mA, NOLOAD tidak akan trip: catat arus bleeder, atur `noload` sesuai. |

## Tahap 4. Kalibrasi lengkap (T3) lalu naik ke 18 V

| # | Perintah / tindakan | Kriteria lolos |
|---|---|---|
| 4.1 | V_IN: PSU 6, 10, 14, 18, 20 V (panduan 4.3), `> raw 10000` di tiap titik + multimeter di terminal board. K1 boleh terbuka (sensor sebelum K1). | R² ≥ 0,999. |
| 4.2 | V_OUT: OL berbeban, Vo 20, 50, 80, 110 V (naikkan Vin ke 12–18 V sesuai kebutuhan, D ≤ 0,66). Titik 150 V menyusul di 4.5. | R² ≥ 0,999. |
| 4.3 | I_IN dan I_OUT: variasi beban (750, 450, 300, 225, 180, 135 Ω) di Vo tetap; acuan probe arus Mean / power analyzer. | R² ≥ 0,999; galat relatif I_OUT terbesar di beban ringan (panduan). |
| 4.4 | `p = polyfit(N, x_ref, 1)` di MATLAB → `> cal vo p1 p2` dst. Cek dengan `raw` sekali lagi. Lalu salin ke `CAL_*` di `qbc_config.h`, compile, flash. | `cfg` menampilkan nilai baru setelah reset. |
| 4.5 | Naik bertahap: Vin 12 → 15 → 18 V, di tiap Vin D 0,3 → 0,4 → 0,5 → 0,6 → 0,654 (`d`, ramp 0,2 /s). Pantau suhu (telemetri `temp`) dan D7 (raba tidak boleh; pakai termometer IR). | Tidak ada fault. Vo di D 0,654 / 18 V / 135 Ω diperkirakan **~121 V** dengan rugi (harness set hw), bukan 150 V ideal. Kalau perlu titik 150 V open-loop: `> dmax 0.70`, `> d 0.70` (perkiraan D tunak dengan rugi 0,70). |

**Siap T4** bila: semua langkah tahap 1–4 lolos, kalibrasi tersimpan di `qbc_config.h`, dan keputusan
D7 (rating 8 A vs perkiraan 8,8 A di titik nominal, panduan 3.2) sudah dibuat. Mulai T4 (Tabel 4.10,
4.11) dan batas CCM–DCM (4.5) mengikuti panduan; ingat `dmax` dan OVP saat beban diperingan.

## Lampiran: closed-loop pertama (sebelum T5/T6)

| # | Perintah | Kriteria lolos |
|---|---|---|
| A.1 | Vin 18 V, beban 135 Ω. `> ctrl hinf`, `> arm`, `> v 50`, `> cl` | Vo naik ke ~50 V tanpa osilasi. `ctrl_cyc` dicatat. `isr_us_max` < 40 µs, `ovr` = 0. |
| A.2 | `> v 100`, lalu `> v 150` | 150 V tercapai. Bandingkan D (TP8 +Duty) dengan Tabel 4.17 (0,703 dengan rugi; harness 0,702). |
| A.3 | `> stop`, `> ctrl mpso`, `> cl`; lalu `pi` | Ketiganya mencapai 150 V. PI ~1,5 s. |
| A.4 | S1 percobaan: `> s 5` dengan osiloskop dipicu dari PE6. `> s -5`. | Perkiraan harness: H∞ overshoot ~0,9 V, tunak 154,65 V (galat tunak 0,35 V); MPSO-PI ~200 ms; PI > 500 ms. |
| A.5 | S2 sesuai panduan: 135 → 270 → 135 Ω. | Harness: puncak 166 V (H∞) sampai 172 V (MPSO-PI), di bawah OVP 180 V. **Jangan** pakai skenario MIL 135 → 101 Ω: dengan rugi, iL1 ~17–20 A → OCP trip (REVIEW, keputusan #K3). |
| A.6 | S3: Vin 18 → 16,2 V dan 18 → 19,8 V, lalu kembali. | Harness: Vin 16,2 → 18 V kembali memberi iL1 puncak 18,1 A (ambang OCP cepat 20 A). Kalau trip di hardware, pakai slew PSU ≥ 10 ms (harness: 17,2 A). |
