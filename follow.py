#!/usr/bin/env python3
"""Person follow for the DJI Mini 4K through the RC-N1C: video in, sticks out.

Pipeline: aoa_gadget.py (video FIFO + stick/FC UDP) -> NVDEC -> YOLO11n person
detector -> controller -> rc_control.Sticks -> ESP32 -> RC stick lines.

Modes (change with commands written to <out>/follow.fifo, one per line):
  idle     sticks not sent (ESP32 holds centre)       [start-up mode]
  hover    all sticks centre
  follow   yaw keeps the target centred; with --distance, pitch keeps its size.
           With --target hand (pose engine): yaw looks at your raised hand, throttle
           keeps the drone level with it, pitch keeps --dist-m from your shoulders
           (+ --reach-m); lower your hand below your elbow and the drone holds.
           While the target is out of view the sticks centre (the drone holds) and
           following resumes when it is seen again.
  land     throttle held down until the drone is down and the motors stop
  start    motors-start stick gesture for 2 s, then hover (props-off test, takeoff)
  takeoff  start, then throttle up to --takeoff-height, then hover
  level    roll the camera wheel until the gimbal reads level (0 deg), then back to the
           previous mode (needs the gimbal angle on --sticks-udp)
  distcal  you stand at --dist-m now: rescale the shoulder-width distance estimate to match
  release  undo an ESP32 hold/land latch (SW1/SW2) so this program drives again
  quit
The ESP32's SW1 button latches all sticks to centre whatever this program sends.

Safety: follow only ever moves yaw (and pitch with --distance), capped at
--yaw-max/--pitch-max; throttle and roll stay centred, so follow can never form
the motor-start/stop gesture. Target lost >0.5 s -> sticks centred until it is
back; a stalled video >0.3 s or stale FC status -> hover. Exiting stops the ESP32 stream and it
centres the sticks within 300 ms.

  sudo .venv/bin/python aoa_gadget.py --ack --video capture/live/video.fifo \\
       --sticks-udp 127.0.0.1:9870 --seconds 3600 --out capture/live &
  .venv/bin/python follow.py --out capture/live            # add --dry to not send sticks
  echo follow > capture/live/follow.fifo
Control page (preview + Land/Hover/Follow/Take off): http://<jetson>:8090/
ESP32 buttons, independent of this program: SW1 = hold (sticks centred),
SW2 = land (throttle down until stopped); both latch until released with R.
"""
import argparse
import csv
import http.server
import math
import os
import queue
import socketserver
import sys
import threading
import time

import cv2

from detector import PersonDetector
from live_view import WARMUP_FRAMES, open_capture
from rc_control import STICK_CENTER, STICK_MAX, STICK_MIN, Esp, Sticks, StickReports

MODES = ("idle", "hover", "follow", "land", "start", "takeoff", "level")
VIDEO_ENDED = 3  # exit code: restart me (run_follow.sh does)


CONTROL_PAGE = b"""<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Mini 4K follow</title><style>
body{margin:0;background:#111;color:#eee;font:16px system-ui;display:flex;flex-direction:column;align-items:center}
img{width:100%;max-width:960px}
.b{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;width:100%;max-width:960px;padding:12px;box-sizing:border-box}
button{font:700 20px system-ui;padding:22px 0;border:0;border-radius:10px;color:#fff;background:#2d4f6b}
#land{background:#b3261e}#hover{background:#8a6d1a}
#msg{min-height:1.4em;padding:0 12px}
</style></head><body>
<img src="/stream" alt="drone camera with detections">
<div class="b"><button id="land">LAND</button><button id="hover">HOVER</button>
<button id="follow">FOLLOW</button><button id="takeoff">TAKE OFF</button></div>
<div id="msg"></div>
<script>
for (const b of document.querySelectorAll("button")) b.onclick = () =>
  fetch("/cmd?c=" + b.id).then(r => r.text()).then(t => document.getElementById("msg").textContent = t)
    .catch(e => document.getElementById("msg").textContent = "not sent: " + e);
</script></body></html>"""


class Preview:
    """Annotated frames as MJPEG (/stream) and a control page (/) with Land/Hover/Follow/Take off."""

    def __init__(self, port, commands):
        self.jpeg = None
        self.cv = threading.Condition()
        preview = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, body, ctype="text/plain"):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path.startswith("/cmd?c="):
                    c = self.path.split("=", 1)[1]
                    if c in ("land", "hover", "follow", "takeoff"):
                        commands.put(c)
                        print(f"{time.strftime('%H:%M:%S')} web: {c}", flush=True)
                        self._send(f"sent {c}".encode())
                    else:
                        self._send(b"unknown command")
                    return
                if self.path != "/stream":
                    self._send(CONTROL_PAGE, "text/html")
                    return
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

        self.pending = None
        self.pending_cv = threading.Condition()
        threading.Thread(target=self._encoder, daemon=True).start()

    def push(self, frame, annotate):
        """Hand over the latest frame; drawing and JPEG encoding happen off the control loop."""
        with self.pending_cv:
            self.pending = (frame, annotate)
            self.pending_cv.notify()

    def _encoder(self):
        while True:
            with self.pending_cv:
                while self.pending is None:
                    self.pending_cv.wait()
                frame, annotate = self.pending
                self.pending = None
            annotate(frame)
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
        self.level_return = "idle"
        self.exit_code = 0
        self.mode_t = time.time()
        self.cmds = queue.Queue()
        self.target = None          # last chosen box
        self.target_t = 0.0
        self.size_goal = None       # box height / frame height to hold (--distance)
        self.ex_f = 0.0             # filtered horizontal error
        self.lost = False
        self.dist_f = 0.0           # filtered relative size error (person) or metres (hand)
        self.ey_f = 0.0             # filtered vertical error (hand)
        self.hand = None            # (x, y) of the raised hand being followed
        self.dist_now = None        # estimated distance to the person, m (hand mode)
        self.lead_state = {}
        self.out = dict(roll=STICK_CENTER, pitch=STICK_CENTER, throttle=STICK_CENTER, yaw=STICK_CENTER, wheel=STICK_CENTER)
        self.preview = Preview(args.port, self.cmds) if args.port else None
        fifo = os.path.join(args.out, "follow.fifo")
        if not os.path.exists(fifo):
            os.mkfifo(fifo)
        threading.Thread(target=self._read_cmds, args=(fifo,), daemon=True).start()
        # line-buffered: readers (the flight scripts' liveness checks) see each frame as it happens
        self.log = csv.writer(open(os.path.join(args.out, f"follow-{time.strftime('%H%M%S')}.csv"), "w", buffering=1))
        self.log.writerow(["t", "mode", "n_people", "cx", "cy", "h", "ex", "size", "roll", "pitch",
                           "throttle", "yaw", "height_m", "fc_state", "fc_flags", "proc_ms"])

    def _read_cmds(self, fifo):
        while True:
            with open(fifo) as f:
                for line in f:
                    if line.strip():
                        self.cmds.put(line.strip().lower())

    def set_mode(self, mode):
        if mode == "level":
            self.level_return = self.mode if self.mode != "level" else "idle"
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

    def lead(self, name, value, now):
        """value + its rate of change x --lead-s: aim where the target will be once the
        command takes effect (video + control delay). Rate is low-passed; 0 = off."""
        if self.a.lead_s <= 0:
            return value
        prev = self.lead_state.get(name)
        rate = 0.0
        if prev is not None and now > prev[0]:
            raw = (value - prev[1]) / (now - prev[0])
            rate = prev[2] + 0.3 * (raw - prev[2])
        self.lead_state[name] = (now, value, rate)
        return max(-1.2, min(1.2, value + rate * self.a.lead_s))

    # ---- hand target (pose engine): COCO keypoints 5/6 shoulders, 7/8 elbows, 9/10 wrists ----
    def hand_of(self, person):
        """The higher raised hand (wrist above its elbow), pushed a little past the wrist."""
        if person is None or len(person) < 6:
            return None
        k = person[5]
        best = None
        for el, wr in ((7, 9), (8, 10)):
            if k[el, 2] < 0.5 or k[wr, 2] < 0.5 or k[wr, 1] > k[el, 1]:
                continue
            hand = (k[wr, 0] + 0.3 * (k[wr, 0] - k[el, 0]), k[wr, 1] + 0.3 * (k[wr, 1] - k[el, 1]))
            if best is None or hand[1] < best[1]:
                best = hand
        return best

    def body_distance(self, person, w):
        """Distance to the person from shoulder width, m (assumes --shoulder-m and --hfov)."""
        k = person[5]
        if k[5, 2] < 0.5 or k[6, 2] < 0.5:
            return None
        px = abs(k[5, 0] - k[6, 0])
        if px < 8:
            return None  # side-on: shoulders overlap, no estimate
        f = (w / 2) / math.tan(math.radians(self.a.hfov) / 2)
        return f * self.a.shoulder_m / px

    def follow_hand(self, o, person, w, h, height, half):
        if self.hand is None:
            return o
        hx, hy = self.hand
        ex = (hx / w - 0.5) * 2
        self.ex_f += self.a.smooth * (ex - self.ex_f)
        ex_l = self.lead("x", self.ex_f, time.time())
        e = 0.0 if abs(ex_l) < self.a.deadband else ex_l
        o["yaw"] = STICK_CENTER + max(-self.a.yaw_max, min(self.a.yaw_max, self.a.k_yaw * e)) * half
        ey = (hy / h - 0.5) * 2                        # >0: hand below centre -> descend
        self.ey_f += self.a.smooth * (ey - self.ey_f)
        ey_l = self.lead("y", self.ey_f, time.time())
        e = 0.0 if abs(ey_l) < self.a.alt_deadband else ey_l
        climb = max(-self.a.thr_max, min(self.a.thr_max, -self.a.k_alt * e))
        if height is not None and ((climb > 0 and height > self.a.ceiling) or (climb < 0 and height < self.a.floor)):
            climb = 0.0
        o["throttle"] = STICK_CENTER + climb * half
        d = self.dist_now = self.body_distance(person, w)
        if d is not None:
            err = d - (self.a.dist_m + self.a.reach_m)  # >0: too far -> forward
            self.dist_f += self.a.dist_smooth * (err - self.dist_f)
            de = 0.0 if abs(self.dist_f) < self.a.dist_deadband_m else self.dist_f
            o["pitch"] = STICK_CENTER + max(-self.a.pitch_max, min(self.a.pitch_max, self.a.k_m * de)) * half
        return o

    def control(self, box, w, h, now):
        o = dict(roll=STICK_CENTER, pitch=STICK_CENTER, throttle=STICK_CENTER, yaw=STICK_CENTER, wheel=STICK_CENTER)
        half = (STICK_MAX - STICK_MIN) / 2
        osd = self.reports.osd
        height = osd["height_m"] if osd else None
        t_mode = now - self.mode_t

        if self.mode == "level":
            # wheel up raises the camera (Mini 4K: ~29 deg/s at full wheel); stop within 1.5 deg of level
            g = self.reports.gimbal
            if g is None or now - g["t"] > 1.0 or t_mode > 8.0:
                print(f"{time.strftime('%H:%M:%S')} level: {'no gimbal angle' if g is None or now - g['t'] > 1.0 else 'timed out'}", flush=True)
                self.set_mode(self.level_return)
            elif abs(g["pitch"]) < 1.5:
                print(f"{time.strftime('%H:%M:%S')} level: gimbal at {g['pitch']:.1f} deg", flush=True)
                self.set_mode(self.level_return)
            else:
                o["wheel"] = STICK_CENTER + max(-1.0, min(1.0, -0.05 * g["pitch"])) * half
            return o
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
        elif self.mode == "follow" and box is not None and self.a.target == "hand":
            return self.follow_hand(o, box, w, h, height, half)
        elif self.mode == "follow" and box is not None:
            cx = (box[0] + box[2]) / 2
            ex = (cx / w - 0.5) * 2                     # -1 (left edge) .. +1 (right edge)
            self.ex_f += self.a.smooth * (ex - self.ex_f)
            ex_l = self.lead("x", self.ex_f, now)
            e = 0.0 if abs(ex_l) < self.a.deadband else ex_l
            o["yaw"] = STICK_CENTER + max(-self.a.yaw_max, min(self.a.yaw_max, self.a.k_yaw * e)) * half
            if self.a.distance and self.size_goal != "off":
                # Distance cues, as fractions of the frame: box height needs head and feet in
                # view; box width (shoulders/arms) still works close up, when height is cut off.
                cut_v = box[1] <= 0.02 * h or box[3] >= 0.98 * h
                cut_h = box[0] <= 0.02 * w or box[2] >= 0.98 * w
                cues = {}
                if not cut_v:
                    cues["height"] = (box[3] - box[1]) / h
                if not cut_h:
                    cues["width"] = (box[2] - box[0]) / w
                if self.size_goal is None:
                    if not cues:
                        print(f"{time.strftime('%H:%M:%S')} target cut off on all sides: no distance hold, yaw only", flush=True)
                        self.size_goal = "off"
                        return o
                    self.size_goal = dict(cues)
                    self.dist_f = 0.0
                    print(f"{time.strftime('%H:%M:%S')} distance hold: keeping "
                          + ", ".join(f"{k} {v:.2f}" for k, v in cues.items()), flush=True)
                cue = next((k for k in ("height", "width") if k in cues and k in self.size_goal), None)
                if cue is None:
                    self.dist_f = 0.0  # no valid cue this frame: hold
                else:
                    rel = (self.size_goal[cue] - cues[cue]) / self.size_goal[cue]  # >0: smaller (farther) -> forward
                    self.dist_f += self.a.dist_smooth * (rel - self.dist_f)
                    de = 0.0 if abs(self.dist_f) < self.a.size_deadband else self.dist_f
                    o["pitch"] = STICK_CENTER + max(-self.a.pitch_max, min(self.a.pitch_max, self.a.k_dist * de)) * half
        return o

    def run(self):
        cap = open_capture(os.path.join(self.a.out, "video.fifo"), *((640, 360) if self.a.small else (None, None)))
        if not cap.isOpened():
            raise SystemExit("video pipeline did not open (is aoa_gadget.py running with --video?)")
        n, last_frame = 0, time.time()
        print("ready: idle. Commands -> " + os.path.join(self.a.out, "follow.fifo"), flush=True)
        try:
            while True:
                ok, frame = cap.read()
                now = time.time()  # frame in hand
                if not ok:
                    # e.g. the drone switched H.264 <-> H.265; a second decoder can't be opened in this
                    # process, so exit with VIDEO_ENDED and let run_follow.sh restart us
                    print("video ended", flush=True)
                    self.exit_code = VIDEO_ENDED
                    break
                stalled = now - last_frame > 0.3
                last_frame = now
                n += 1
                while not self.cmds.empty():
                    c = self.cmds.get()
                    if c == "quit":
                        return
                    if c == "distcal":  # you are standing at --dist-m right now: rescale the estimate to match
                        if self.dist_now:
                            self.a.shoulder_m *= self.a.dist_m / self.dist_now
                            print(f"{time.strftime('%H:%M:%S')} distcal: read {self.dist_now:.2f} m at a true {self.a.dist_m} m"
                                  f" -> shoulder width {self.a.shoulder_m:.3f} m", flush=True)
                        else:
                            print("distcal: no distance estimate right now (shoulders and a raised hand must be visible)", flush=True)
                        continue
                    if c == "release":  # undo an ESP32 SW1/SW2/H/G latch
                        print(self.sticks.esp.resume() if self.sticks else "dry: no ESP32", flush=True)
                        continue
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
                if self.a.target == "hand":
                    self.hand = self.hand_of(box)
                    self.dist_now = None
                found = (self.hand is not None) if self.a.target == "hand" else (box is not None)
                if found:
                    self.target, self.target_t = box, now

                osd = self.reports.osd
                if self.mode == "follow":
                    fc_stale = osd is None or now - osd["t"] > 1.0
                    if stalled or fc_stale:
                        print(f"{time.strftime('%H:%M:%S')} {'video stalled' if stalled else 'FC status stale'} -> hover", flush=True)
                        self.set_mode("hover")
                    else:
                        lost = now - self.target_t > self.a.lost_s
                        if lost != self.lost:  # sticks centre (drone holds) while lost, resume on reacquire
                            print(f"{time.strftime('%H:%M:%S')} target {'lost: holding' if lost else 'reacquired'}", flush=True)
                            self.lost = lost
                self.out = self.control(box if box is not None and now - self.target_t < 0.2 else None, w, h, now)

                if self.sticks and self.mode != "idle":
                    self.sticks.set(**self.out)
                elif self.sticks:
                    self.sticks.esp.release()
                proc_ms = (time.time() - now) * 1000  # frame in hand -> sticks sent

                if self.a.target == "hand":
                    size = f"{self.dist_now:.2f}" if self.dist_now else ""   # metres in hand mode
                else:
                    size = (box[3] - box[1]) / h if box is not None else ""
                self.log.writerow([f"{now:.3f}", self.mode, len(people),
                                   *((self.hand[0], self.hand[1], "") if self.a.target == "hand" and self.hand else
                                     ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2, box[3] - box[1]) if box and self.a.target != "hand"
                                     else ("", "", "")),
                                   f"{self.ex_f:.3f}", size, *[round(self.out[k]) for k in ("roll", "pitch", "throttle", "yaw")],
                                   osd["height_m"] if osd else "", osd["state"] if osd else "", osd["flags"] if osd else "",
                                   f"{proc_ms:.1f}"])
                if self.preview and n % 2 == 0:
                    snap = (list(people), box, osd, dict(self.out), self.hand, self.dist_now, self.mode)
                    self.preview.push(frame, lambda f, snap=snap: self.draw(f, *snap))
        finally:
            cap.release()  # before CUDA goes away, or nvvidconv errors on the way out
            if self.sticks:
                self.sticks.esp.close()

    def draw(self, frame, people, box, osd, out, hand, dist_now, mode):
        for b in people:
            col = (0, 255, 0) if b is box else (160, 160, 160)
            cv2.rectangle(frame, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), col, 2)
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        if self.a.target == "hand":
            cv2.line(frame, (0, h // 2), (w, h // 2), (255, 255, 255), 1)
            if hand is not None:
                cv2.circle(frame, (int(hand[0]), int(hand[1])), 18, (0, 200, 255), 4)
            if dist_now:
                cv2.putText(frame, f"{dist_now:.2f} m", (10, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
        txt = (f"{mode.upper()}{'  MOTORS' if osd and osd.get('motors_on') else ''}  yaw {out['yaw']:.0f} pitch {out['pitch']:.0f} "
               f"thr {out['throttle']:.0f}  h {osd['height_m'] if osd else '?'} m  {osd['state'] if osd else ''}"
               + (f"  bat {self.reports.battery['percent']}%" if self.reports.battery else "")
               + ("  DRY" if self.a.dry else ""))
        cv2.rectangle(frame, (0, 0), (w, 34), (0, 0, 0), -1)
        cv2.putText(frame, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default="capture/live", help="gadget output dir (video.fifo) and log dir")
    ap.add_argument("--target", choices=("person", "hand"), default="person")
    ap.add_argument("--engine", help="default: yolo11n-fp16 for person, yolo11n-pose-fp16 for hand")
    ap.add_argument("--cal", default="calibration/sticks.json")
    ap.add_argument("--udp", type=int, default=9870)
    ap.add_argument("--port", type=int, default=8090, help="MJPEG preview port, 0 = off")
    ap.add_argument("--dry", action="store_true", help="compute and log, never send sticks")
    ap.add_argument("--full-res", dest="small", action="store_false",
                    help="decode to 1280x720 instead of 640x360 (the detector sees 640 px either way)")
    ap.add_argument("--conf", type=float, default=0.45)
    ap.add_argument("--k-yaw", type=float, default=0.6, help="yaw stick fraction per unit of horizontal error")
    ap.add_argument("--yaw-max", type=float, default=0.35, help="yaw stick cap, fraction of full deflection")
    ap.add_argument("--deadband", type=float, default=0.08, help="horizontal error ignored, fraction of half-width")
    ap.add_argument("--smooth", type=float, default=0.5, help="error low-pass factor per frame")
    ap.add_argument("--lead-s", type=float, default=0.0,
                    help="aim this far ahead along the target's motion (offsets video delay; try 0.15-0.3)")
    ap.add_argument("--distance", action="store_true", help="also hold distance with pitch (stretch goal)")
    ap.add_argument("--k-dist", type=float, default=1.0, help="pitch fraction per relative size change")
    ap.add_argument("--pitch-max", type=float, default=0.2)
    ap.add_argument("--size-deadband", type=float, default=0.08, help="relative size change ignored")
    ap.add_argument("--dist-smooth", type=float, default=0.3, help="size-error low-pass factor per frame")
    ap.add_argument("--lost-s", type=float, default=0.5)
    # hand target
    ap.add_argument("--dist-m", type=float, default=1.75, help="distance to keep, m (hand mode)")
    ap.add_argument("--reach-m", type=float, default=0.0, help="added to --dist-m: how far the hand is in front of the shoulders")
    ap.add_argument("--shoulder-m", type=float, default=0.40, help="assumed shoulder width, m")
    ap.add_argument("--hfov", type=float, default=73.7, help="camera horizontal field of view, deg")
    ap.add_argument("--k-m", type=float, default=0.3, help="pitch fraction per metre of distance error")
    ap.add_argument("--dist-deadband-m", type=float, default=0.15)
    ap.add_argument("--k-alt", type=float, default=0.6, help="throttle fraction per unit of vertical error")
    ap.add_argument("--thr-max", type=float, default=0.3, help="throttle cap, fraction of full deflection")
    ap.add_argument("--alt-deadband", type=float, default=0.1)
    ap.add_argument("--ceiling", type=float, default=2.5, help="no climbing above this reported height, m")
    ap.add_argument("--floor", type=float, default=0.3, help="no descending below this reported height, m")
    ap.add_argument("--takeoff-height", type=float, default=1.0, help="m")
    ap.add_argument("--takeoff-throttle", type=float, default=0.5, help="fraction of full up")
    ap.add_argument("--takeoff-timeout", type=float, default=4.0, help="s of climbing before giving up")
    args = ap.parse_args()
    if not args.engine:
        args.engine = "models/yolo11n-pose-fp16.engine" if args.target == "hand" else "models/yolo11n-fp16.engine"
    f = Follower(args)
    f.run()
    sys.exit(f.exit_code)


if __name__ == "__main__":
    main()
