"""Jetson side of the ESP32 stick driver.

- StickReports: the RC's own stick report, streamed by `aoa_gadget.py --sticks-udp`.
- Esp: USB serial link to the ESP32 firmware (firmware/esp32_sticks), with keepalive.
- Sticks: command positions in the RC's units (364..1684, centre 1024) through
  the calibration made by calibrate_sticks.py.
"""
import bisect
import glob
import json
import socket
import threading
import time

import serial

CHANNELS = ("roll", "pitch", "throttle", "yaw", "wheel")  # ESP32 outputs, in order
ESP_PINS = (2, 4, 6, 8, 10)  # GPIO of each channel (firmware PWM_PINS)
DUTY_MAX = 4095
STICK_MIN, STICK_CENTER, STICK_MAX = 364, 1024, 1684


def find_port():
    ports = sorted(glob.glob("/dev/serial/by-id/*Espressif*")) or sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        raise SystemExit("ESP32 not found: no /dev/serial/by-id/*Espressif* or /dev/ttyACM*")
    return ports[0]


class StickReports:
    """Latest RC stick report (~9 Hz) and flight-controller status (~10 Hz) from the gadget's UDP stream."""

    def __init__(self, port=9870, host="127.0.0.1"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.latest = None
        self.count = 0
        self.osd = None  # latest FC status (duml.parse_osd_general + "t")
        self.cv = threading.Condition()
        threading.Thread(target=self._rx, daemon=True).start()

    def _rx(self):
        while True:
            msg = json.loads(self.sock.recv(4096))
            if msg.get("type") == "osd":
                self.osd = msg
                continue
            with self.cv:
                self.latest = msg
                self.count += 1
                self.cv.notify_all()

    def wait_new(self, n=1, timeout=3.0):
        """Block until n reports newer than the call have arrived; return the last."""
        with self.cv:
            target = self.count + n
            if not self.cv.wait_for(lambda: self.count >= target, timeout):
                raise TimeoutError("no stick reports: is aoa_gadget.py running with --sticks-udp?")
            return self.latest


class Esp:
    """Serial link to the ESP32. Keeps re-sending the last duties so its timeout doesn't fire."""

    def __init__(self, port=None, rate_hz=50):
        # DTR/RTS must stay released: on the ESP32-S3's built-in USB they drive the chip's reset
        self.ser = serial.Serial()
        self.ser.port = port or find_port()
        self.ser.baudrate = 115200
        self.ser.timeout = 0.5
        self.ser.write_timeout = 1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(0.5)  # opening the port restarts the S3; let it boot, drop its boot log
        self.ser.reset_input_buffer()
        self.duty = None  # None = send nothing, the ESP32 falls back to neutral
        self.lock = threading.Lock()
        self.period = 1.0 / rate_hz
        self.running = True
        threading.Thread(target=self._keepalive, daemon=True).start()

    def _keepalive(self):
        while self.running:
            with self.lock:
                if self.duty is not None:
                    self.ser.write(("D " + " ".join(str(int(d)) for d in self.duty) + "\n").encode())
            time.sleep(self.period)

    def _cmd(self, line):
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write((line + "\n").encode())
            return self.ser.readline().decode(errors="replace").strip()

    def set_duty(self, duties):
        with self.lock:
            self.duty = [max(0, min(DUTY_MAX, int(round(d)))) for d in duties]

    def release(self):
        """Stop sending; the ESP32 returns to neutral after its timeout."""
        with self.lock:
            self.duty = None

    def set_neutral(self, duties):
        return self._cmd("N " + " ".join(str(int(round(d))) for d in duties))

    def hold(self):
        return self._cmd("H")

    def resume(self):
        return self._cmd("R")

    def status(self):
        return self._cmd("?")

    def close(self):
        self.running = False
        self.release()
        time.sleep(self.period * 2)
        self.ser.close()


class Sticks:
    """Calibrated stick commands in the RC's own units."""

    def __init__(self, esp, cal_path="calibration/sticks.json"):
        self.esp = esp
        cal = json.load(open(cal_path))
        self.cal = cal["outputs"]  # list in ESP output order
        self.by_name = {c["report_channel"]: i for i, c in enumerate(self.cal) if c.get("report_channel")}

    def duty_for(self, i, value):
        c = self.cal[i]
        if not c.get("points") or value == STICK_CENTER:
            return c.get("centre_duty", DUTY_MAX / 2)  # middle of the RC's centre dead zone
        vals = [v for _, v in c["points"]]  # increasing
        duts = [d for d, _ in c["points"]]
        value = max(vals[0], min(vals[-1], value))
        k = bisect.bisect_left(vals, value)
        if k == 0:
            return duts[0]
        v0, v1, d0, d1 = vals[k - 1], vals[k], duts[k - 1], duts[k]
        return d0 + (d1 - d0) * (value - v0) / (v1 - v0) if v1 != v0 else d0

    def set(self, **values):
        """set(roll=1200, throttle=1024, ...) in 364..1684; unnamed channels centre."""
        duties = []
        for i, c in enumerate(self.cal):
            name = c.get("report_channel")
            duties.append(self.duty_for(i, values.get(name, STICK_CENTER)))
        self.esp.set_duty(duties)

    def set_normalized(self, **values):
        """set_normalized(roll=0.2, throttle=-1.0, ...) in -1..1."""
        half = (STICK_MAX - STICK_MIN) / 2
        self.set(**{k: STICK_CENTER + max(-1.0, min(1.0, v)) * half for k, v in values.items()})

    def centre(self):
        self.set()
