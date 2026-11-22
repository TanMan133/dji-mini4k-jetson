#!/usr/bin/env python3
"""Fly one session through a running follow.py: take off, follow, land.

Checks first that follow.py is live (fresh log rows at ~30 fps) and that the drone is on
the ground with its motors off. Then: takeoff -> follow until --follow-s (0 = no limit),
SW2 / LAND (ESP32 latch or control page), or battery <= --min-battery, then land.

  ./run_follow.sh --out capture/live --target hand &
  python3 tools/fly.py --out capture/live --follow-s 0
"""
import argparse
import csv
import glob
import io
import os
import subprocess
import sys
import time


def latest_row(path, header):
    """Last complete row of a follow.py CSV that is still being written."""
    with open(path, "rb") as f:
        f.seek(0, 2)
        f.seek(max(0, f.tell() - 8000))
        lines = f.read().decode(errors="replace").splitlines()[1:]  # the first may be partial
    rows = [r for r in csv.reader(io.StringIO("\n".join(lines))) if len(r) == len(header)]
    return dict(zip(header, rows[-1])) if rows else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="capture/live", help="follow.py --out directory")
    ap.add_argument("--follow-s", type=float, default=0, help="follow time before landing, 0 = until you land it")
    ap.add_argument("--min-battery", type=int, default=25, help="land at this battery %%")
    args = ap.parse_args()

    fifo = os.path.join(args.out, "follow.fifo")
    log = max(glob.glob(os.path.join(args.out, "follow-*.csv")), key=os.path.getmtime)
    header = next(csv.reader(open(log)))

    def send(cmd):
        subprocess.run(["timeout", "3", "sh", "-c", f"echo {cmd} > {fifo}"])
        print(time.strftime("%H:%M:%S"), ">>>", cmd, flush=True)

    r = latest_row(log, header)
    with open(log) as f:
        recent = sum(1 for row in csv.DictReader(f) if float(row["t"]) > time.time() - 3)
    if r is None or time.time() - float(r["t"]) > 0.5 or recent < 45:
        sys.exit(f"follow.py is not live ({log}): start run_follow.sh first")
    if float(r["height_m"] or 0) > 0.3 or int(r["fc_flags"] or 0) & 0x08:
        sys.exit("the drone is not on the ground with its motors off")
    if r.get("battery") and int(r["battery"]) <= args.min_battery + 10:
        sys.exit(f"battery {r['battery']} %: charge first")

    send("takeoff")
    t0 = time.time()
    phase, t_phase, last_print = "takeoff", t0, 0.0
    while True:
        r = latest_row(log, header)
        now = time.time()
        motors = int(r["fc_flags"] or 0) & 0x08
        bat = int(r["battery"]) if r.get("battery") else None
        if now - last_print > 5:
            print(f"{time.strftime('%H:%M:%S')} {r['mode']:7s} h {r['height_m']:>4} bat {bat}% "
                  f"yaw {r['yaw']} thr {r['throttle']} pitch {r['pitch']} {r.get('hand_reason', '')}", flush=True)
            last_print = now
        if phase == "takeoff":
            if now - t0 > 12 and not motors:
                print("motors never started (check DJI Fly for a take-off warning)", flush=True)
                send("land")
                phase = "land"
            elif r["mode"] == "hover":
                send("follow")
                phase, t_phase = "follow", now
        elif phase == "follow":
            if bat is not None and bat <= args.min_battery:
                print(f"battery {bat} %: landing", flush=True)
                send("land")
                phase = "land"
            elif args.follow_s and now - t_phase > args.follow_s:
                send("hover")
                time.sleep(2)
                send("land")
                phase = "land"
            elif not motors and now - t_phase > 2:  # SW2 landed it
                print("motors stopped (SW2)", flush=True)
                send("land")
                phase = "land"
            elif r["mode"] in ("land", "idle") and now - t_phase > 2:  # LAND on the control page
                phase = "land"
        elif r["mode"] == "idle" and not motors:
            print(f"{time.strftime('%H:%M:%S')} landed, motors off; {now - t0:.0f} s", flush=True)
            return
        time.sleep(0.3)


if __name__ == "__main__":
    main()
