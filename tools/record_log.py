#!/usr/bin/env python3
"""record_log.py — record one trial from the ESP32 to a CSV file (task J-1.3).

Usage (from the repository root, board plugged in, nothing else using the port):
    python3 tools/record_log.py rest            # 60 s, condition "rest"
    python3 tools/record_log.py trem5 -s 30     # 30 s
    python3 tools/record_log.py trem5 -p /dev/cu.usbserial-0001

What it does
  1. Opens the serial port (921600 baud). This resets the ESP32: keep the
     arm STILL for the 2 s gyro calibration, until "ready" is printed.
  2. Sends "LOG on", writes every CSV line to data/raw/<date>_<time>_<name>.csv
     (data/raw/ is not committed; copy the files to the Drive).
  3. Sends "LOG off" and "STATUS", then checks the file: lost samples (gaps
     in the counter n), sample rate, worst loop time.

Requires pyserial:  python3 -m pip install pyserial
"""

import argparse
import datetime as dt
import glob
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing: python3 -m pip install pyserial")

BAUD = 921600


def find_port():
    cands = sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.SLAB_USBtoUART*") +
                   glob.glob("/dev/cu.wchusbserial*") + glob.glob("/dev/ttyUSB*") +
                   glob.glob("/dev/ttyACM*"))
    if len(cands) == 1:
        return cands[0]
    if not cands:
        sys.exit("No serial port found. Is the board plugged in (data cable, not charge-only)?\n"
                 "If nothing appears in /dev/cu.*, install the CP210x driver (see docs/bringup.md).")
    sys.exit("Several ports found, choose one with -p:\n  " + "\n  ".join(cands))


def wait_ready(ser, timeout):
    t_end = time.time() + timeout
    while time.time() < t_end:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print("  " + line)
        if line.startswith("# ready"):
            return True
        if "FAULT" in line:
            return False
    return False


def check(path):
    """Return (n_lines, n_lost, rate_hz, loop_max_us)."""
    with open(path) as f:
        header = f.readline().strip().split(",")
        i_n, i_t, i_loop = header.index("n"), header.index("t_us"), header.index("loop_us")
        prev_n, lost, rows, bad = None, 0, 0, 0
        t_first = t_last = None
        loop_max = 0
        for line in f:
            parts = line.strip().split(",")
            if len(parts) != len(header):
                bad += 1
                continue
            n, t = int(parts[i_n]), int(parts[i_t])
            if prev_n is not None and n != prev_n + 1:
                lost += n - prev_n - 1
            prev_n = n
            t_first = t if t_first is None else t_first
            t_last = t
            loop_max = max(loop_max, int(parts[i_loop]))
            rows += 1
    dur = ((t_last - t_first) % 2**32) / 1e6 if rows > 1 else 0
    rate = (rows - 1) / dur if dur > 0 else 0
    return rows, lost, bad, rate, loop_max


def main():
    ap = argparse.ArgumentParser(description="Record one trial from the ESP32.")
    ap.add_argument("name", help="condition, e.g. rest, vol, trem4, trem5, trem6, trem5vol, glass")
    ap.add_argument("-s", "--seconds", type=float, default=60.0)
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("-o", "--outdir", default="data/raw")
    args = ap.parse_args()

    port = args.port or find_port()
    os.makedirs(args.outdir, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(args.outdir, f"{stamp}_{args.name}.csv")

    print(f"Port {port}. Keep the arm STILL (gyro calibration)...")
    ser = serial.Serial(port, BAUD, timeout=0.5)
    if not wait_ready(ser, timeout=10):
        ser.close()
        sys.exit("The board did not report 'ready'. Check the wiring (run SCAN in the monitor).")

    input(f"\nReady to record '{args.name}' for {args.seconds:.0f} s. Press Enter to START...")
    ser.reset_input_buffer()
    ser.write(b"LOG on\n")
    n = 0
    t_end = None
    with open(path, "w") as out:
        while True:
            line = ser.readline().decode(errors="replace")
            if not line:
                continue
            if line.startswith("#"):
                print("  " + line.strip())
                continue
            if line.startswith("n,"):
                out.write(line)
                t_end = time.time() + args.seconds
                print("Recording...")
                continue
            if t_end is None:
                continue
            out.write(line)
            n += 1
            if n % 2500 == 0:
                print(f"  {n / 500:.0f} s")
            if time.time() >= t_end:
                break
    ser.write(b"LOG off\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"STATUS\n")
    time.sleep(0.3)
    for line in ser.read_all().decode(errors="replace").splitlines():
        if line.startswith("#"):
            print("  " + line)
    ser.close()

    rows, lost, bad, rate, loop_max = check(path)
    print(f"\nSaved {path}")
    print(f"  {rows} samples, {rate:.1f} Hz, lost {lost}, malformed lines {bad}, "
          f"worst loop {loop_max} us")
    ok = lost == 0 and bad == 0 and abs(rate - 500) < 2 and loop_max < 2000
    print("  OK" if ok else "  PROBLEM: see docs/bringup.md, section 'If something goes wrong'")


if __name__ == "__main__":
    main()
