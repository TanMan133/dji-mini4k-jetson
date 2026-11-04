#!/usr/bin/env python3
"""Person follow for the DJI Mini 4K through the RC-N1C: video in, sticks out.

Pipeline: aoa_gadget.py (video FIFO + stick/FC UDP) -> NVDEC -> YOLO11n person
detector -> controller -> rc_control.Sticks -> ESP32 -> RC stick lines.

Modes (change with commands written to <out>/follow.fifo, one per line):
  idle     sticks not sent (ESP32 holds centre)       [start-up mode]
  hover    all sticks centre
  follow   yaw keeps the target centred; with --distance, pitch keeps its size
  land     throttle held down until the drone is down and the motors stop
  start    motors-start stick gesture for 2 s, then hover (props-off test, takeoff)
  takeoff  start, then throttle up to --takeoff-height, then hover
  quit
The ESP32's SW1 button latches all sticks to centre whatever this program sends.

Safety: follow only ever moves yaw (and pitch with --distance), capped at
--yaw-max/--pitch-max; throttle and roll stay centred, so follow can never form
the motor-start/stop gesture. Lost target >0.5 s, a stalled video >0.3 s or
stale FC status while airborne -> hover. Exiting stops the ESP32 stream and it
centres the sticks within 300 ms.

  sudo .venv/bin/python aoa_gadget.py --ack --video capture/live/video.fifo \\
       --sticks-udp 127.0.0.1:9870 --seconds 3600 --out capture/live &
  .venv/bin/python follow.py --out capture/live            # add --dry to not send sticks
  echo follow > capture/live/follow.fifo
Preview: http://<jetson>:8090/
"""
import argparse
import csv
import http.server
import os
import queue
import socketserver
import threading
import time

import cv2

from detector import PersonDetector
from live_view import WARMUP_FRAMES, open_capture
from rc_control import STICK_CENTER, STICK_MAX, STICK_MIN, Esp, Sticks, StickReports

MODES = ("idle", "hover", "follow", "land", "start", "takeoff")


class Preview:
    """Latest annotated frame as MJPEG over HTTP."""

    def __init__(self, port):
        self.jpeg = None
        self.cv = threading.Condition()
        preview = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
                self.end_headers()
                try:
                    while True:
                        with preview.cv:
                            preview.cv.wait(1.0)
                            jpg = preview.jpeg
                        if jpg:
                            self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        srv = socketserver.ThreadingTCPServer(("0.0.0.0", port), Handler)
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()

    def push(self, frame):
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self.cv:
                self.jpeg = jpg.tobytes()
                self.cv.notify_all()


class Follower:
    def __init__(self, args):
        self.a = args
        self.reports = StickReports(args.udp)
        self.sticks = None if args.dry else Sticks(Esp(), args.cal)
        self.det = PersonDetector(args.engine, conf=args.conf)
        self.mode = "idle"
        self.mode_t = time.time()
        self.cmds = queue.Queue()
        self.target = None          # last chosen box
        self.target_t = 0.0
        self.size_goal = None       # box height / frame height to hold (--distance)
        self.ex_f = 0.0             # filtered horizontal error
        self.out = dict(roll=STICK_CENTER, pitch=STICK_CENTER, throttle=STICK_CENTER, yaw=STICK_CENTER)
        self.preview = Preview(args.port) if args.port else None
        fifo = os.path.join(args.out, "follow.fifo")
        if not os.path.exists(fifo):
            os.mkfifo(fifo)
        threading.Thread(target=self._read_cmds, args=(fifo,), daemon=True).start()
        self.log = csv.writer(open(os.path.join(args.out, f"follow-{time.strftime('%H%M%S')}.csv"), "w"))
        self.log.writerow(["t", "mode", "n_people", "cx", "cy", "h", "ex", "size", "roll", "pitch",
                           "throttle", "yaw", "height_m", "fc_state", "fc_flags"])

    def _read_cmds(self, fifo):
        while True:
            with open(fifo) as f:
                for line in f:
                    if line.strip():
                        self.cmds.put(line.strip().lower())

    def set_mode(self, mode):
        if mode != self.mode:
            print(f"{time.strftime('%H:%M:%S')} mode {self.mode} -> {mode}", flush=True)
        self.mode, self.mode_t = mode, time.time()
        if mode == "follow":
            self.size_goal = None  # take the target's current size as the distance to hold

    # ---- target selection: stick with the person nearest the last one ----
    def pick(self, people, w, h):
        if not people:
            return None
        if self.target is None or time.time() - self.target_t > 1.0:
            return people[0]  # most confident
        tx, ty = (self.target[0] + self.target[2]) / 2, (self.target[1] + self.target[3]) / 2
        return min(people, key=lambda b: ((b[0] + b[2]) / 2 - tx) ** 2 + ((b[1] + b[3]) / 2 - ty) ** 2)

    def control(self, box, w, h, now):
        o = dict(roll=STICK_CENTER, pitch=STICK_CENTER, throttle=STICK_CENTER, yaw=STICK_CENTER)
        half = (STICK_MAX - STICK_MIN) / 2
        osd = self.reports.osd
        height = osd["height_m"] if osd else None
        t_mode = now - self.mode_t

        if self.mode in ("start", "takeoff") and t_mode < 2.0:
            # motors-start gesture: both sticks down and outward
            o.update(throttle=STICK_MIN, yaw=STICK_MIN, pitch=STICK_MIN, roll=STICK_MAX)
        elif self.mode == "start":
            self.set_mode("hover")
        elif self.mode == "takeoff":
            climbing = t_mode < 2.0 + self.a.takeoff_timeout
            if height is not None and height < self.a.takeoff_height and climbing:
                o["throttle"] = STICK_CENTER + self.a.takeoff_throttle * half
            else:
                self.set_mode("hover")
        elif self.mode == "land":
            # gentle descent, full down near the floor so the drone lands and stops its motors
            if osd and not osd.get("motors_on", True) and t_mode > 1.0:
                self.set_mode("idle")  # down and stopped
            else:
                gentle = height is not None and height > 0.5
                o["throttle"] = STICK_CENTER - 0.5 * half if gentle else STICK_MIN
        elif self.mode == "follow" and box is not None:
            cx = (box[0] + box[2]) / 2
            ex = (cx / w - 0.5) * 2                     # -1 (left edge) .. +1 (right edge)
            self.ex_f += self.a.smooth * (ex - self.ex_f)
            e = 0.0 if abs(self.ex_f) < self.a.deadband else self.ex_f
            o["yaw"] = STICK_CENTER + max(-self.a.yaw_max, min(self.a.yaw_max, self.a.k_yaw * e)) * half
            if self.a.distance:
                size = (box[3] - box[1]) / h
                if self.size_goal is None:
                    self.size_goal = size
                de = self.size_goal - size               # >0: target smaller (farther) -> forward
                de = 0.0 if abs(de) < self.a.size_deadband else de
                o["pitch"] = STICK_CENTER + max(-self.a.pitch_max, min(self.a.pitch_max, self.a.k_dist * de)) * half
        return o

    def run(self):
        cap = open_capture(os.path.join(self.a.out, "video.fifo"))
        if not cap.isOpened():
            raise SystemExit("video pipeline did not open (is aoa_gadget.py running with --video?)")
        n, last_frame = 0, time.time()
        print("ready: idle. Commands -> " + os.path.join(self.a.out, "follow.fifo"), flush=True)
        try:
            while True:
                ok, frame = cap.read()
                now = time.time()
                if not ok:
                    print("video ended", flush=True)
                    break
                stalled = now - last_frame > 0.3
                last_frame = now
                n += 1
                while not self.cmds.empty():
                    c = self.cmds.get()
                    if c == "quit":
                        return
                    osd = self.reports.osd
                    if c in ("start", "takeoff") and (osd is None or osd["height_m"] > 0.3):
                        # the start gesture in the air is DJI's emergency motor stop
                        print(f"refusing {c}: height {'unknown' if osd is None else osd['height_m']} m", flush=True)
                    elif c in MODES:
                        self.set_mode(c)
                    else:
                        print(f"unknown command {c!r}", flush=True)
                if n <= WARMUP_FRAMES:
                    continue
                h, w = frame.shape[:2]
                people = self.det(frame)
                box = self.pick(people, w, h)
                if box is not None:
                    self.target, self.target_t = box, now

                osd = self.reports.osd
                if self.mode == "follow":
                    lost = now - self.target_t > self.a.lost_s
                    fc_stale = osd is None or now - osd["t"] > 1.0
                    if lost or stalled or fc_stale:
                        why = "target lost" if lost else ("video stalled" if stalled else "FC status stale")
                        print(f"{time.strftime('%H:%M:%S')} {why} -> hover", flush=True)
                        self.set_mode("hover")
                self.out = self.control(box if box is not None and now - self.target_t < 0.2 else None, w, h, now)

                if self.sticks and self.mode != "idle":
                    self.sticks.set(**self.out)
                elif self.sticks:
                    self.sticks.esp.release()

                size = (box[3] - box[1]) / h if box is not None else ""
                self.log.writerow([f"{now:.3f}", self.mode, len(people),
                                   *(((box[0] + box[2]) / 2, (box[1] + box[3]) / 2, box[3] - box[1]) if box else ("", "", "")),
                                   f"{self.ex_f:.3f}", size, *[round(self.out[k]) for k in ("roll", "pitch", "throttle", "yaw")],
                                   osd["height_m"] if osd else "", osd["state"] if osd else "", osd["flags"] if osd else ""])
                if self.preview and n % 2 == 0:
                    self.draw(frame, people, box, osd)
                    self.preview.push(frame)
        finally:
            cap.release()  # before CUDA goes away, or nvvidconv errors on the way out
            if self.sticks:
                self.sticks.esp.close()

    def draw(self, frame, people, box, osd):
        for b in people:
            col = (0, 255, 0) if box is not None and b == box else (160, 160, 160)
            cv2.rectangle(frame, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), col, 2)
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        txt = (f"{self.mode.upper()}{'  MOTORS' if osd and osd.get('motors_on') else ''}  yaw {self.out['yaw']:.0f} pitch {self.out['pitch']:.0f} "
               f"thr {self.out['throttle']:.0f}  h {osd['height_m'] if osd else '?'} m  {osd['state'] if osd else ''}"
               + ("  DRY" if self.a.dry else ""))
        cv2.rectangle(frame, (0, 0), (w, 34), (0, 0, 0), -1)
        cv2.putText(frame, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default="capture/live", help="gadget output dir (video.fifo) and log dir")
    ap.add_argument("--engine", default="models/yolo11n-fp16.engine")
    ap.add_argument("--cal", default="calibration/sticks.json")
    ap.add_argument("--udp", type=int, default=9870)
    ap.add_argument("--port", type=int, default=8090, help="MJPEG preview port, 0 = off")
    ap.add_argument("--dry", action="store_true", help="compute and log, never send sticks")
    ap.add_argument("--conf", type=float, default=0.45)
    ap.add_argument("--k-yaw", type=float, default=0.6, help="yaw stick fraction per unit of horizontal error")
    ap.add_argument("--yaw-max", type=float, default=0.35, help="yaw stick cap, fraction of full deflection")
    ap.add_argument("--deadband", type=float, default=0.08, help="horizontal error ignored, fraction of half-width")
    ap.add_argument("--smooth", type=float, default=0.5, help="error low-pass factor per frame")
    ap.add_argument("--distance", action="store_true", help="also hold distance with pitch (stretch goal)")
    ap.add_argument("--k-dist", type=float, default=1.5)
    ap.add_argument("--pitch-max", type=float, default=0.2)
    ap.add_argument("--size-deadband", type=float, default=0.04)
    ap.add_argument("--lost-s", type=float, default=0.5)
    ap.add_argument("--takeoff-height", type=float, default=1.0, help="m")
    ap.add_argument("--takeoff-throttle", type=float, default=0.5, help="fraction of full up")
    ap.add_argument("--takeoff-timeout", type=float, default=4.0, help="s of climbing before giving up")
    args = ap.parse_args()
    Follower(args).run()


if __name__ == "__main__":
    main()
