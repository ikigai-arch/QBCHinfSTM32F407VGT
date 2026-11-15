/* hil_stm32_v2.ino -- Firmware board QBC v2 (H-inf / PI-FR / MPSO-PI) untuk uji T1-T6.
 * Target : STM32F407VGT (DevEBox) -- Arduino IDE + STM32duino (v0.9.0 dikompilasi dengan core 3.1.0;
 *          v0.9.1 hanya mengubah file portabel, belum dikompilasi ulang dengan arduino-cli)
 * Setelan: Board "Generic STM32F4 series", Board part number "Generic F407VGTx",
 *          USB support "CDC (generic 'Serial' supersede U(S)ART)", Optimize "Smallest (-Os default)".
 *
 * Isi folder sketch (semua harus satu folder bernama hil_stm32_v2):
 *   hil_stm32_v2.ino   ini: setup/loop, baris perintah USB, keluaran non-blocking
 *   qbc_core.c/.h      mesin status, proteksi, relai, ramp, logging (C portabel, diuji di PC)
 *   qbc_config.h       semua ambang dengan alasan
 *   ctrl_mux.c/.h      pemilih H-inf / PI / MPSO-PI + anti-windup H-inf
 *   hw_f407.c/.h       TIM1 + ADC1 injected + ISR + watchdog (register-level)
 *   qbc_sim.c/.h       model rata-rata QBC untuk mode SIM (uji meja tanpa daya)
 *   controller_step.c/.h, controller_coeffs.h   H-inf (TIDAK diubah, md5 = pil_stm32)
 *   pi_ctrl.c/.h, pi_params.h                   PI & MPSO-PI (TIDAK diubah, md5 = PIL_PI_MPSO)
 *   test_pc/           harness uji di PC (diabaikan Arduino IDE; lihat test_pc/README.md)
 *
 * Saat boot: PWM mati (PA8 LOW), K1/K2 terbuka, kipas mati, status OFF. Daya hanya bisa naik lewat
 * perintah eksplisit "arm" lalu "ol"/"cl". Daftar perintah: ketik "help" atau lihat PERINTAH_SERIAL_v2.md.
 * Kontrol berjalan di ISR ADC (dipicu TIM1), BUKAN di loop(); loop() hanya mengurus teks USB.
 */
#include <Arduino.h>

extern "C" {
#include "qbc_core.h"
#include "qbc_config.h"
#include "hw_f407.h"
}

/* ---------- keluaran teks non-blocking ----------
 * USBSerial::write() di STM32duino menunggu sampai antrean muat selama DTR aktif, dan
 * "if (Serial)" berisi delay(10). Keduanya tidak dipakai di sini: teks masuk ring buffer dan
 * dikirim hanya sebanyak availableForWrite(). Jika buffer penuh, teks dibuang (dihitung). */
static char     txb[4096];
static uint32_t txh = 0, txt = 0, txdrop = 0;

static void tx_put(const char *s, int n)
{
    for (int i = 0; i < n; i++) {
        uint32_t nx = (txh + 1u) % sizeof(txb);
        if (nx == txt) { txdrop++; return; }
        txb[txh] = s[i];
        txh = nx;
    }
}
static void tx_puts(const char *s) { tx_put(s, (int)strlen(s)); }
static uint32_t tx_free(void) { return (txt + sizeof(txb) - txh - 1u) % sizeof(txb); }

static void tx_flush(void)
{
    if (!Serial.dtr()) { txt = txh; return; }            /* tidak ada terminal: buang */
    while (txt != txh) {
        int room = Serial.availableForWrite();
        if (room <= 0) return;
        uint32_t chunk = (txh >= txt) ? (txh - txt) : (sizeof(txb) - txt);
        if (chunk > (uint32_t)room) chunk = (uint32_t)room;
        Serial.write((const uint8_t *)&txb[txt], chunk);
        txt = (txt + chunk) % sizeof(txb);
    }
}

/* ---------- baris perintah ---------- */
static char rxl[80];
static uint8_t rxn = 0;
static char rep[1400];

static void handle_line(void)
{
    rxl[rxn] = 0;
    if (qbc_command(rxl, rep, sizeof(rep))) tx_puts(rep);
    rxn = 0;
}

static void poll_rx(void)
{
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;
        if (c == '!' && rxn == 0) {                      /* darurat: langsung, tanpa menunggu Enter */
            rxl[0] = '!'; rxn = 1; handle_line();
            continue;
        }
        if (c == '\n' || c == '\r') { if (rxn > 0) handle_line(); }
        else if (rxn < sizeof(rxl) - 1u) rxl[rxn++] = (char)c;
    }
}

void setup()
{
    hw_safe_gpio_early();          /* PA8, K1, K2, kipas = LOW sebelum apa pun */
    Serial.begin(115200);          /* baud diabaikan USB CDC */
    qbc_init();
    hw_init();                     /* TIM1 jalan dengan MOE = 0 (PWM LOW), ADC + ISR aktif */
    hw_iwdg_start();
    tx_puts("\r\n# " QBC_FW_VERSION "\r\n# reset: ");
    tx_puts(hw_reset_cause());
    tx_puts(hw_clock_is_hse() == 1 ? "  clock: HSE 168 MHz\r\n" : "  clock: HSI (HSE GAGAL) - cek kristal\r\n");
    tx_puts("# PWM OFF, K1/K2 terbuka. Ketik help.\r\n");
    tx_puts("# T,ms,state,ctrl,flags,vref,vo,vin,iin,iout,temp,duty,vo_min,vo_max,faults,isr_us,isr_us_max,ctrl_cyc,ovr,sat_ms\r\n");
}

void loop()
{
    static char line[256];
    static uint32_t last_ms = 0;
    static int last_dtr = -1;
    uint32_t now = millis();

    hw_service(now);               /* watchdog + deteksi ISR macet */

    int dtr = Serial.dtr() ? 1 : 0;
    if (dtr != last_dtr) {
        qbc_set_link(dtr);
        if (dtr && last_dtr == 0) tx_puts("# link USB tersambung\r\n");
        last_dtr = dtr;
    }

    poll_rx();

    /* event (prioritas), lalu keluaran raw/dump, lalu telemetri */
    int n;
    while (tx_free() > 300u && (n = qbc_poll_event(line, sizeof(line))) > 0) tx_put(line, n);
    for (int k = 0; k < 8 && tx_free() > 300u; k++) {
        if ((n = qbc_poll_output(line, sizeof(line))) <= 0) break;
        tx_put(line, n);
    }
    if (tx_free() > 300u && (n = qbc_poll_telemetry(now, line, sizeof(line))) > 0) tx_put(line, n);

    if (txdrop && tx_free() > 100u) {
        char t[48];
        int m = snprintf(t, sizeof(t), "# %lu byte teks dibuang (USB lambat)\r\n", (unsigned long)txdrop);
        txdrop = 0;
        tx_put(t, m);
    }
    tx_flush();
    (void)last_ms;
}
