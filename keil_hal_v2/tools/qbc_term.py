#!/usr/bin/env python3
"""qbc_term.py -- terminal serial untuk firmware QBC v2 (keil_hal_v2, USART3 lewat USB-TTL).

Fungsi:
  * mengirim heartbeat ("\\r" tiap 200 ms) dan menyalakan "hb 1000" di firmware, supaya firmware
    melakukan DISARM bila PC/terminal mati atau kabel USB-TTL lepas (pengganti DTR di versi Arduino);
  * menampilkan event (E,...) dan balasan perintah; telemetri (T,...) ditampilkan ringkas tiap 1 s;
  * menyimpan SEMUA baris ke log_<tanggal>.txt, telemetri ke tel_<tanggal>.csv, dan hasil
    "capdump" ke cap_<tanggal>_<n>.csv.

Pemakaian:  pip install pyserial
            python qbc_term.py COM5            (Windows; lihat nomor COM di Device Manager)
            python qbc_term.py /dev/ttyUSB0    (Linux)
Ketik perintah lalu Enter (lihat PERINTAH_SERIAL_v2.md). Ketik "!" lalu Enter = DARURAT.
Ketik "quit" untuk keluar (firmware akan DISARM sendiri sekitar 2 s kemudian bila daya aktif).
"""
import sys, threading, time, datetime
import serial  # pyserial

BAUD = 115200
HB_PERIOD = 0.2

def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    port = serial.Serial(sys.argv[1], BAUD, timeout=0.05)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    flog = open(f"log_{stamp}.txt", "w", encoding="utf-8")
    ftel = open(f"tel_{stamp}.csv", "w", encoding="utf-8")
    ftel.write("t,ms,state,ctrl,flags,vref,vo,vin,iin,iout,temp,duty,vo_min,vo_max,faults,isr_us,isr_us_max,ctrl_cyc,ovr,sat_ms\n")
    lock = threading.Lock()
    stop = threading.Event()
    state = {"cap": None, "ncap": 0, "last_tel_print": 0.0}

    def send(s):
        with lock:
            port.write(s.encode("ascii", "replace"))

    def reader():
        buf = b""
        while not stop.is_set():
            buf += port.read(4096)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip("\r")
                if not line:
                    continue
                flog.write(f"{time.time():.3f} {line}\n")
                if line.startswith("T,"):
                    ftel.write(f"{time.time():.3f},{line[2:]}\n")
                    now = time.time()
                    if now - state["last_tel_print"] >= 1.0:
                        state["last_tel_print"] = now
                        f = line.split(",")
                        if len(f) >= 15:
                            print(f"  [{f[2]:7s} {f[3]:4s}] vref={f[5]} vo={f[6]} vin={f[7]} iin={f[8]} "
                                  f"iout={f[9]} T={f[10]} d={f[11]} fault={f[14]}")
                    continue
                if line.startswith("C,HDR"):
                    state["ncap"] += 1
                    state["cap"] = open(f"cap_{stamp}_{state['ncap']}.csv", "w", encoding="utf-8")
                    state["cap"].write("# " + line + "\n")
                    print(line)
                    continue
                if line.startswith("C,") and state["cap"]:
                    if line.startswith("C,END"):
                        state["cap"].close(); state["cap"] = None
                        print(f"  capture tersimpan: cap_{stamp}_{state['ncap']}.csv")
                    else:
                        state["cap"].write(line[2:] + "\n")
                    continue
                print(line)

    def heartbeat():
        while not stop.is_set():
            send("\r")
            time.sleep(HB_PERIOD)

    threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=heartbeat, daemon=True).start()
    time.sleep(0.3)
    send("hb 1000\r")
    print(f"Tersambung ke {sys.argv[1]} @ {BAUD}. Heartbeat aktif (hb 1000). Log: log_{stamp}.txt")
    try:
        while True:
            cmd = input()
            if cmd.strip().lower() == "quit":
                break
            if cmd.strip() == "!":
                send("!")
                continue
            send(cmd + "\r")
    except (KeyboardInterrupt, EOFError):
        pass
    stop.set()
    time.sleep(0.2)
    flog.close(); ftel.close()
    port.close()

if __name__ == "__main__":
    main()
