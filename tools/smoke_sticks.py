#!/usr/bin/env python3
"""Quick check of the ESP32 -> RC stick wiring before calibrating.

With every output at mid-scale, then each output nudged low and high in turn,
prints what the RC reports, so a dead, swapped or reversed channel shows up
in a few seconds. Needs the gadget running with --sticks-udp.

  sudo python3 aoa_gadget.py --ack --sticks-udp 127.0.0.1:9870 --seconds 600 --out capture/smoke &
  python3 tools/smoke_sticks.py
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0])
from rc_control import CHANNELS, DUTY_MAX, ESP_PINS, Esp, StickReports  # noqa: E402

LOW, HIGH = DUTY_MAX // 4, DUTY_MAX * 3 // 4


def read(reports, esp, duties):
    esp.set_duty(duties)
    time.sleep(0.05)
    r = reports.wait_new(2)
    return {c: r[c] for c in CHANNELS}


def main():
    reports = StickReports()
    print("before the ESP32 drives anything:", {c: reports.wait_new(1)[c] for c in CHANNELS})
    esp = Esp()
    time.sleep(0.5)  # opening the port restarts the ESP32
    mid = [DUTY_MAX // 2] * len(CHANNELS)
    try:
        base = read(reports, esp, mid)
        print("all outputs mid-scale:          ", base)
        for k, name in enumerate(CHANNELS):
            lo = read(reports, esp, mid[:k] + [LOW] + mid[k + 1:])
            hi = read(reports, esp, mid[:k] + [HIGH] + mid[k + 1:])
            moved = {c: (lo[c], hi[c]) for c in CHANNELS if abs(hi[c] - lo[c]) > 20}
            if not moved:
                verdict = "NO RESPONSE"
            elif list(moved) != [name]:
                verdict = f"moves {', '.join(moved)} (expected {name})"
            else:
                verdict = "ok, " + ("reversed" if hi[name] < lo[name] else "same direction")
            print(f"IO{ESP_PINS[k]:<2} {name:8s} duty {LOW}->{HIGH}: "
                  + ("  ".join(f"{c} {a}->{b}" for c, (a, b) in moved.items()) or "-") + f"   [{verdict}]")
        read(reports, esp, mid)
    finally:
        esp.close()


if __name__ == "__main__":
    main()
