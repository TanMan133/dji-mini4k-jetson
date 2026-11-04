#!/usr/bin/env python3
"""Android Open Accessory (AOA) device-side gadget for the DJI RC-N1.

The RC-N1 is the USB *host* on its phone port and speaks AOA. This turns the
Jetson's USB-C port (tegra-xudc) into a fake Android phone with configfs +
FunctionFS, answers the AOA handshake (51/52/53), re-enumerates as an accessory
(18d1:2d00) and logs everything the RC sends on the bulk OUT endpoint.

Run as root. Unbinds NVIDIA's l4t USB gadget while running, rebinds it on exit.

  sudo python3 aoa_gadget.py --seconds 120 --out capture/
"""
import argparse
import json
import collections
import fcntl
import os
import queue
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import duml

UDC = "3550000.usb"
GADGETS = "/sys/kernel/config/usb_gadget"
G = f"{GADGETS}/dji_aoa"
L4T = f"{GADGETS}/l4t"
FFS_NAME = "dji"
FFS_MNT = "/dev/ffs-dji"

# FunctionFS ABI (include/uapi/linux/usb/functionfs.h)
DESC_MAGIC_V2, STR_MAGIC = 3, 2
HAS_FS, HAS_HS, ALL_CTRL_RECIP, CONFIG0_SETUP = 1, 2, 64, 128
EVENTS = {0: "BIND", 1: "UNBIND", 2: "ENABLE", 3: "DISABLE", 4: "SETUP", 5: "SUSPEND", 6: "RESUME"}

# AOA vendor requests
GET_PROTOCOL, SEND_STRING, START = 51, 52, 53
AOA_STRINGS = ["manufacturer", "model", "description", "version", "uri", "serial"]

GOOGLE_VID, PHONE_PID, ACCESSORY_PID = 0x18D1, 0x4EE1, 0x2D00


def log(*a):
    print(f"{time.strftime('%H:%M:%S')}", *a, flush=True)


def wr(path, val):
    with open(path, "w") as f:
        f.write(val)


def rd(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def fmt(p):
    return (f"{p['src']:02x}->{p['dst']:02x} set {p['set']:02x} id {p['id']:02x} type {p['type']:02x} "
            f"seq {p['seq']:5d} len {len(p['payload']):3d}: {p['payload'][:32].hex(' ')}")


class VideoOut:
    """Write the live view (H.264 or H.265, whichever the drone sends) to a FIFO
    for a decoder, dropping data while no reader is attached. A new reader is
    started on a frame boundary, preceded by the latest parameter sets, so it
    decodes at once instead of waiting up to ~9 s for the next ones. The codec
    ("h264"/"h265") is written to <fifo>.codec for the reader's parser.

    The Mini 4K sent H.265 until it was paired with DJI Fly on iOS, then H.264."""

    PARAM_SETS = (("h265", b"\x00\x00\x00\x01\x40\x01"),   # VPS
                  ("h264", b"\x00\x00\x00\x01\x67"),        # SPS (nal_ref_idc 3)
                  ("h264", b"\x00\x00\x00\x01\x27"))        # SPS (nal_ref_idc 1)

    def __init__(self, path):
        self.path = path
        self.fd = None
        self.params = b""
        self.codec = None
        self.next_open = 0
        if path and not os.path.exists(path):
            os.mkfifo(path, 0o666)
        if path:
            os.chmod(path, 0o666)
            if os.path.exists(path + ".codec"):
                os.remove(path + ".codec")  # stale from an earlier session

    def _find_params(self, chunk):
        """Return the offset of a parameter-set run in chunk (caching it), or -1."""
        for codec, sig in self.PARAM_SETS:
            i = chunk.find(sig)
            if i >= 0:
                break
        else:
            return -1
        j = i
        while True:
            k = chunk.find(b"\x00\x00\x01", j + 4)
            if k < 0 or k + 3 >= len(chunk):
                return -1  # the run continues into the next chunk; wait for the next one
            nal = chunk[k + 3]
            is_param = ((nal >> 1) & 0x3F) in (32, 33, 34) if codec == "h265" else (nal & 0x1F) in (7, 8)
            if not is_param:
                self.params = chunk[i:k - 1 if chunk[k - 1] == 0 else k]
                break
            j = k
        if codec != self.codec:
            self.codec = codec
            log(f"live view is {codec}")
            if self.path:
                with open(self.path + ".codec", "w") as f:
                    f.write(codec)
                os.chmod(self.path + ".codec", 0o666)
        return i

    def feed(self, chunk):
        i = self._find_params(chunk)
        if not self.path:
            return
        if self.fd is None:
            if not chunk.startswith(b"\x00\x00\x00\x01") or not self.params or time.time() < self.next_open:
                return
            try:
                self.fd = os.open(self.path, os.O_WRONLY | os.O_NONBLOCK)
            except OSError:  # ENXIO: no reader yet
                self.next_open = time.time() + 0.5
                return
            fcntl.fcntl(self.fd, 1031, 1 << 20)  # F_SETPIPE_SZ: 1 MiB of slack
            log("video reader attached")
            if i != 0:
                chunk = self.params + chunk
        try:
            os.write(self.fd, chunk)
        except BlockingIOError:
            pass  # reader is behind; GDR heals the dropped chunk within ~12 frames
        except OSError:
            log("video reader detached")
            os.close(self.fd)
            self.fd = None


def ffs_descriptors():
    intf = struct.pack("<9B", 9, 4, 0, 0, 2, 0xFF, 0xFF, 0x00, 1)

    def ep(addr, mps):
        return struct.pack("<BBBBHB", 7, 5, addr, 2, mps, 0)

    fs = intf + ep(0x81, 64) + ep(0x02, 64)  # ep1 = bulk IN, ep2 = bulk OUT
    hs = intf + ep(0x81, 512) + ep(0x02, 512)
    flags = HAS_FS | HAS_HS | ALL_CTRL_RECIP | CONFIG0_SETUP
    body = struct.pack("<III", flags, 3, 3) + fs + hs
    return struct.pack("<II", DESC_MAGIC_V2, 8 + len(body)) + body


def ffs_strings():
    s = struct.pack("<H", 0x0409) + b"Android Accessory Interface\0"
    return struct.pack("<IIII", STR_MAGIC, 16 + len(s), 1, 1) + s


class Gadget:
    def __init__(self, pid, out_dir):
        self.pid = pid
        self.out_dir = out_dir
        self.l4t_udc = rd(f"{L4T}/UDC")
        self.aoa = {"protocol_queries": 0, "strings": {}, "started": False}
        self.stop = threading.Event()
        self.rx_pid = 0
        self.ack = False
        self.video_path = None
        self.rec_video = False
        self.sticks_udp = None  # (host, port) for stick-report JSON
        self.txrec = open(os.path.join(out_dir, "phone_to_rc.rec"), "wb", buffering=0)
        self.raw = open(os.path.join(out_dir, "rc_to_phone.bin"), "wb", buffering=0)
        self.rec = open(os.path.join(out_dir, "rc_to_phone.rec"), "wb", buffering=0)  # <d t><I len><data>

    # ---- configfs ----
    def setup(self):
        if self.l4t_udc:
            log(f"unbinding l4t gadget from {self.l4t_udc}")
            wr(f"{L4T}/UDC", "\n")
        os.makedirs(f"{G}/strings/0x409", exist_ok=True)
        wr(f"{G}/idVendor", hex(GOOGLE_VID))
        wr(f"{G}/idProduct", hex(self.pid))
        wr(f"{G}/bcdUSB", "0x0200")
        wr(f"{G}/bcdDevice", "0x0100")
        wr(f"{G}/max_speed", "high-speed")
        wr(f"{G}/strings/0x409/manufacturer", "Google")
        wr(f"{G}/strings/0x409/product", "Pixel")
        wr(f"{G}/strings/0x409/serialnumber", "JETSONAOA0001")
        os.makedirs(f"{G}/configs/c.1/strings/0x409", exist_ok=True)
        wr(f"{G}/configs/c.1/strings/0x409/configuration", "accessory")
        wr(f"{G}/configs/c.1/MaxPower", "2")
        os.makedirs(f"{G}/functions/ffs.{FFS_NAME}", exist_ok=True)
        if not os.path.islink(f"{G}/configs/c.1/ffs.{FFS_NAME}"):
            os.symlink(f"{G}/functions/ffs.{FFS_NAME}", f"{G}/configs/c.1/ffs.{FFS_NAME}")
        os.makedirs(FFS_MNT, exist_ok=True)
        if not os.path.ismount(FFS_MNT):
            subprocess.run(["mount", "-t", "functionfs", FFS_NAME, FFS_MNT], check=True)
        self.ep0 = os.open(f"{FFS_MNT}/ep0", os.O_RDWR)
        os.write(self.ep0, ffs_descriptors())
        os.write(self.ep0, ffs_strings())
        os.set_blocking(self.ep0, False)
        self.bind()

    def bind(self):
        wr(f"{G}/UDC", UDC)
        log(f"bound as {GOOGLE_VID:04x}:{rd(f'{G}/idProduct')[2:]:0>4} on {UDC}")

    def unbind(self):
        if rd(f"{G}/UDC"):
            wr(f"{G}/UDC", "\n")

    def teardown(self):
        self.stop.set()
        self.stop_rx()
        try:
            self.unbind()
        except OSError as e:
            log("unbind:", e)
        try:
            os.close(self.ep0)
        except (OSError, AttributeError):
            pass
        subprocess.run(["umount", FFS_MNT], check=False)
        for p in [f"{G}/configs/c.1/ffs.{FFS_NAME}"]:
            if os.path.islink(p):
                os.unlink(p)
        for d in [f"{G}/configs/c.1/strings/0x409", f"{G}/configs/c.1",
                  f"{G}/functions/ffs.{FFS_NAME}", f"{G}/strings/0x409", G]:
            try:
                os.rmdir(d)
            except OSError:
                pass
        if self.l4t_udc:
            try:
                wr(f"{L4T}/UDC", self.l4t_udc)
                log(f"rebound l4t gadget to {self.l4t_udc}")
            except OSError as e:
                log("could not rebind l4t gadget:", e)
        self.raw.close()
        self.rec.close()
        self.txrec.close()
        with open(os.path.join(self.out_dir, "aoa_handshake.json"), "w") as f:
            json.dump(self.aoa, f, indent=2)

    # ---- AOA re-enumeration ----
    def reenumerate_as_accessory(self):
        time.sleep(0.2)  # let the START status stage finish
        log("re-enumerating as accessory 18d1:2d00")
        self.stop_rx()
        self.unbind()
        wr(f"{G}/idProduct", hex(ACCESSORY_PID))
        time.sleep(0.3)
        self.bind()
        self.start_rx()

    # ---- ep0 control handling ----
    def handle_setup(self, rtype, req, wval, widx, wlen):
        tag = f"SETUP type=0x{rtype:02x} req={req} wValue=0x{wval:04x} wIndex={widx} wLength={wlen}"
        vendor = (rtype & 0x60) == 0x40
        if rtype & 0x80:  # device-to-host
            if vendor and req == GET_PROTOCOL:
                self.aoa["protocol_queries"] += 1
                os.write(self.ep0, struct.pack("<H", 2)[:wlen])
                log(tag, "-> GET_PROTOCOL, answered 2")
                return
            log(tag, "-> stall")
            try:
                os.read(self.ep0, 1)  # wrong-direction op on ep0 = stall
            except OSError:
                pass
            return
        data = os.read(self.ep0, wlen)
        if vendor and req == SEND_STRING:
            name = AOA_STRINGS[widx] if widx < len(AOA_STRINGS) else f"idx{widx}"
            self.aoa["strings"][name] = data.rstrip(b"\0").decode(errors="replace")
            log(tag, f"-> SEND_STRING {name} = {self.aoa['strings'][name]!r}")
        elif vendor and req == START:
            self.aoa["started"] = True
            log(tag, "-> START accessory")
            self.reenumerate_as_accessory()
        else:
            log(tag, f"-> acked, data={data.hex()}")

    def poll_ep0(self, timeout_s):
        # ep0 is O_NONBLOCK: a blocking ep0 read sleeps holding ffs->mutex and
        # deadlocks a UDC unbind.
        if not self.poller.poll(timeout_s * 1000):
            return
        try:
            buf = os.read(self.ep0, 12 * 4)
        except BlockingIOError:
            return
        for i in range(0, len(buf), 12):
            rtype, req, wval, widx, wlen, etype = struct.unpack_from("<BBHHHB", buf, i)
            name = EVENTS.get(etype, etype)
            if name == "SETUP":
                try:
                    self.handle_setup(rtype, req, wval, widx, wlen)
                except OSError as e:
                    log(f"setup req={req} failed:", e)
            else:
                log("event", name)

    # ---- bulk OUT (RC -> phone), in a child process ----
    # A sync FunctionFS read can't be cancelled from another thread and blocks
    # unbind; SIGKILL on a child interrupts it cleanly.
    def start_rx(self):
        pid = os.fork()
        if pid:
            self.rx_pid = pid
            return
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        fd = os.open(f"{FFS_MNT}/ep2", os.O_RDONLY)
        txq = queue.Queue()
        threading.Thread(target=self.tx_loop, args=(txq,), daemon=True).start()
        threading.Thread(target=self.cmd_loop, args=(txq,), daemon=True).start()
        video = VideoOut(self.video_path)
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self.sticks_udp else None
        seen = set()
        total, reads, last, t_last = 0, 0, 0, time.time()
        while True:
            try:
                data = os.read(fd, 65536)
            except OSError as e:
                log("rx:", e)
                time.sleep(0.05)
                continue
            if not data:
                continue
            t = time.time()
            total += len(data)
            reads += 1
            has_video = False
            for ch, f in duml.blocks(data):
                if ch == duml.CH_VIDEO:
                    has_video = True
                    video.feed(f)
                    continue
                p = duml.parse(f)
                if not p:
                    log(f"RX non-DUML {len(f)} B: {f[:48].hex(' ')}")
                    continue
                if udp and p["src"] == 0x06 and p["set"] == 0x06 and p["id"] == 0x05:
                    st = duml.parse_sticks(p["payload"])
                    st.update(type="sticks", t=t)
                    udp.sendto(json.dumps(st).encode(), self.sticks_udp)
                elif udp and p["set"] == 0x03 and p["id"] == 0x43 and len(p["payload"]) >= 36:
                    osd = duml.parse_osd_general(p["payload"])
                    osd.update(type="osd", t=t)
                    udp.sendto(json.dumps(osd).encode(), self.sticks_udp)
                key = (p["src"], p["dst"], p["set"], p["id"], p["type"] & 0x80)
                is_new = key not in seen
                if is_new or (p["type"] & 0x80 and p["src"] == 0x06):
                    seen.add(key)
                    log(f"RX {'new ' if is_new else ''}{fmt(p)}")
                if self.ack and p["type"] & 0x60 and not p["type"] & 0x80:
                    txq.put(duml.build(p["dst"], p["src"], p["seq"], 0x80, p["set"], p["id"], b"\x00"))
            if has_video and not self.rec_video:
                # keep the DUML blocks of a mixed read, drop the video
                data = b"".join(duml.wrap(f) for ch, f in duml.blocks(data) if ch != duml.CH_VIDEO)
            if data:
                self.raw.write(data)
                self.rec.write(struct.pack("<dI", t, len(data)) + data)
            if t - t_last >= 2:
                log(f"rx total {total} B in {reads} reads ({(total - last) * 8 / (t - t_last) / 1e6:.2f} Mbit/s)")
                last, t_last = total, t

    def cmd_loop(self, txq):
        """Inject frames written to <out>/cmd.fifo, one per line: dst set id type payload_hex."""
        path = os.path.join(self.out_dir, "cmd.fifo")
        if not os.path.exists(path):
            os.mkfifo(path, 0o666)
        os.chmod(path, 0o666)
        seq = 0x4000
        while True:
            with open(path) as f:
                for line in f:
                    parts = line.split()
                    if len(parts) < 4:
                        continue
                    dst, cset, cid, ctype = (int(x, 16) for x in parts[:4])
                    payload = bytes.fromhex("".join(parts[4:]))
                    seq += 1
                    log(f"CMD dst {dst:02x} set {cset:02x} id {cid:02x} type {ctype:02x} seq {seq} payload {payload.hex(' ')}")
                    txq.put(duml.build(0x02, dst, seq, ctype, cset, cid, payload))

    def tx_loop(self, txq):
        fd = os.open(f"{FFS_MNT}/ep1", os.O_WRONLY)
        shown = collections.Counter()
        while True:
            frame = txq.get()
            try:
                os.write(fd, duml.wrap(frame))
            except OSError as e:
                log("tx:", e)
                continue
            self.txrec.write(struct.pack("<dI", time.time(), len(frame)) + frame)
            p = duml.parse(frame)
            k = (p["dst"], p["set"], p["id"])
            if shown[k] < 2:
                shown[k] += 1
                log(f"TX {fmt(p)}")

    def stop_rx(self):
        if self.rx_pid:
            os.kill(self.rx_pid, signal.SIGKILL)
            os.waitpid(self.rx_pid, 0)
            self.rx_pid = 0

    def run(self, seconds):
        self.poller = select.poll()
        self.poller.register(self.ep0, select.POLLIN)
        self.start_rx()
        t_end = time.time() + seconds
        while not self.stop.is_set() and time.time() < t_end:
            self.poll_ep0(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--out", default="capture")
    ap.add_argument("--ack", action="store_true", help="acknowledge RC frames that request an ack")
    ap.add_argument("--video", metavar="FIFO", help="write the HEVC live view to this named pipe")
    ap.add_argument("--rec-video", action="store_true",
                    help="also keep video blocks in rc_to_phone.rec (~25 MB/min)")
    ap.add_argument("--sticks-udp", metavar="HOST:PORT",
                    help="send each RC stick report and FC status as JSON to this UDP address, e.g. 127.0.0.1:9870")
    ap.add_argument("--accessory", action="store_true",
                    help="enumerate directly as 18d1:2d00 (skip the handshake)")
    args = ap.parse_args()
    if os.geteuid() != 0:
        sys.exit("run as root")
    os.makedirs(args.out, exist_ok=True)

    g = Gadget(ACCESSORY_PID if args.accessory else PHONE_PID, args.out)
    g.ack = args.ack
    g.video_path = args.video
    g.rec_video = args.rec_video
    if args.sticks_udp:
        host, port = args.sticks_udp.rsplit(":", 1)
        g.sticks_udp = (host, int(port))
    signal.signal(signal.SIGTERM, lambda *_: g.stop.set())
    signal.signal(signal.SIGINT, lambda *_: g.stop.set())
    try:
        g.setup()
        g.run(args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        g.teardown()
        log(f"done: {os.path.getsize(os.path.join(args.out, 'rc_to_phone.bin'))} B captured, handshake={json.dumps(g.aoa)}")


if __name__ == "__main__":
    main()
