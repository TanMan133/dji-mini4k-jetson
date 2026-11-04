#!/usr/bin/env python3
"""Calibrate the ESP32 stick outputs against the RC's own stick report.

Sweeps each ESP32 output (IO2, IO4, IO6, IO8, IO10) across its PWM range while the others sit
at mid-scale, reads what the RC reports, and saves a duty -> stick-value table
per output plus the duty that centres each stick. The centre duties are also
stored on the ESP32 as its neutral (what it falls back to when commands stop).

Drone OFF for this: the stick report comes from the RC alone.

  sudo python3 aoa_gadget.py --ack --sticks-udp 127.0.0.1:9870 --seconds 900 --out capture/cal &
  python3 calibrate_sticks.py
"""
import argparse
import json
import os
import time

from rc_control import CHANNELS, DUTY_MAX, ESP_PINS, STICK_CENTER, STICK_MAX, STICK_MIN, Esp, Sticks, StickReports

MIN_RESPONSE = 30  # stick counts; less than this over a full sweep = output not connected


def sweep(esp, reports, k, step, base):
    rows = []
    for d in list(range(0, DUTY_MAX + 1, step)) + [DUTY_MAX]:
        duties = list(base)
        duties[k] = d
        esp.set_duty(duties)
        time.sleep(0.05)
        r = reports.wait_new(2)  # the first may predate the change
        rows.append((d, {c: r[c] for c in CHANNELS}))
    return rows


def build_table(rows, ch):
    """Monotonic (duty, value) points, increasing in value, plateaus trimmed."""
    pts = [(d, v[ch]) for d, v in rows]
    sign = 1 if pts[-1][1] >= pts[0][1] else -1
    kept = [pts[0]]
    for d, v in pts[1:]:
        if sign * (v - kept[-1][1]) > 0:
            kept.append((d, v))
    if sign < 0:
        kept.reverse()
    return kept


def interp_duty(points, value):
    for (d0, v0), (d1, v1) in zip(points, points[1:]):
        if v0 <= value <= v1:
            return d0 + (d1 - d0) * (value - v0) / (v1 - v0) if v1 != v0 else d0
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="ESP32 serial port (default: auto)")
    ap.add_argument("--udp", type=int, default=9870, help="stick-report UDP port")
    ap.add_argument("--step", type=int, default=64, help="PWM duty step for the sweep")
    ap.add_argument("--out", default="calibration/sticks.json")
    args = ap.parse_args()

    reports = StickReports(args.udp)
    print("rest:", {c: reports.wait_new(1)[c] for c in CHANNELS})
    esp = Esp(args.port)
    print(esp.status())

    base = [DUTY_MAX // 2] * len(CHANNELS)
    outputs = []
    try:
        for k, name in enumerate(CHANNELS):
            print(f"\nIO{ESP_PINS[k]} ({name}): sweeping...", flush=True)
            rows = sweep(esp, reports, k, args.step, base)
            spans = {c: max(v[c] for _, v in rows) - min(v[c] for _, v in rows) for c in CHANNELS}
            ch = max(spans, key=spans.get)
            if spans[ch] < MIN_RESPONSE:
                print(f"  no response (largest change {spans[ch]} on {ch}): not connected or not analog")
                outputs.append({"esp_pin": ESP_PINS[k], "intended": name, "report_channel": None,
                                "points": [], "centre_duty": DUTY_MAX // 2, "raw": rows})
                continue
            if ch != name:
                print(f"  NOTE: IO{ESP_PINS[k]} moves '{ch}', not '{name}' (wires swapped?) - using '{ch}'")
            pts = build_table(rows, ch)
            lo, hi = pts[0][1], pts[-1][1]
            # the RC has a dead zone around centre: use the middle of the duties that read exactly 1024
            dead = [d for d, v in rows if v[ch] == STICK_CENTER]
            centre = (min(dead) + max(dead)) / 2 if dead else interp_duty(pts, STICK_CENTER)
            if dead:
                print(f"  dead zone: duty {min(dead)}..{max(dead)} all read {STICK_CENTER}")
            print(f"  -> {ch}: reaches {lo}..{hi}, centre at duty {centre and round(centre)}"
                  f"{'' if lo <= STICK_MIN and hi >= STICK_MAX else '  (does not cover the full 364..1684)'}")
            outputs.append({"esp_pin": ESP_PINS[k], "intended": name, "report_channel": ch, "points": pts,
                            "centre_duty": round(centre) if centre is not None else DUTY_MAX // 2,
                            "min": lo, "max": hi, "raw": rows})
            esp.set_duty(base)

        cal = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "step": args.step, "outputs": outputs}
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        json.dump(cal, open(args.out, "w"), indent=1)
        print(f"\nsaved {args.out}")
        print(esp.set_neutral([o["centre_duty"] for o in outputs]))

        # verification: each calibrated channel to min / centre / max
        print("\ncheck (target -> reported):")
        sticks = Sticks(esp, args.out)
        for o in outputs:
            ch = o["report_channel"]
            if not ch:
                continue
            got = []
            for target in (STICK_MIN, STICK_CENTER, STICK_MAX):
                sticks.set(**{ch: target})
                time.sleep(0.05)
                got.append(f"{target}->{reports.wait_new(2)[ch]}")
            print(f"  {ch:8s} " + "  ".join(got))
        sticks.centre()
        time.sleep(0.3)
    finally:
        esp.close()


if __name__ == "__main__":
    main()
