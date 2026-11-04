"""DJI DUML v1 framing + the RC-N1C "logiclink" USB wrapper (55 cc 49 57 <le32 len>)."""
import struct

WRAP_MAGIC = b"\x55\xcc\x49\x57"
WRAP_BLOCK = 512  # the RC pads every wrapped frame to a multiple of 512 B
CH_DUML, CH_VIDEO = 0x49, 0x4A  # third byte of the wrapper magic


def _crc8_table():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
        t.append(c)
    return t


def _crc16_table():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
        t.append(c)
    return t


_T8, _T16 = _crc8_table(), _crc16_table()


def crc8(data, seed=0x77):
    c = seed
    for b in data:
        c = _T8[(c ^ b) & 0xFF]
    return c


def crc16(data, seed=0x3692):
    c = seed
    for b in data:
        c = (c >> 8) ^ _T16[(c ^ b) & 0xFF]
    return c


def build(src, dst, seq, cmd_type, cmd_set, cmd_id, payload=b""):
    n = 13 + len(payload)
    hdr = struct.pack("<BH", 0x55, n | (1 << 10))
    hdr += bytes([crc8(hdr)])
    body = hdr + struct.pack("<BBHBBB", src, dst, seq, cmd_type, cmd_set, cmd_id) + payload
    return body + struct.pack("<H", crc16(body))


def parse(f):
    """-> dict or None if the frame is malformed / CRC fails."""
    if len(f) < 13 or f[0] != 0x55:
        return None
    n = struct.unpack_from("<H", f, 1)[0] & 0x3FF
    if n > len(f) or crc8(f[:3]) != f[3] or crc16(f[:n - 2]) != struct.unpack_from("<H", f, n - 2)[0]:
        return None
    src, dst, seq, ctype, cset, cid = struct.unpack_from("<BBHBBB", f, 4)
    return dict(src=src, dst=dst, seq=seq, type=ctype, set=cset, id=cid, payload=f[11:n - 2], raw=f[:n])


def wrap(frame, pad=True):
    w = WRAP_MAGIC + struct.pack("<I", len(frame)) + frame
    if pad:
        w += b"\0" * (-len(w) % WRAP_BLOCK)
    return w


def blocks(buf):
    """Yield (channel, body) for each 55 cc <ch> 57 block in one USB read.

    Channel 0x49 carries one DUML frame per block, 0x4a carries the live view
    (HEVC Annex B, split into <=2040 B chunks)."""
    j = 0
    while j + 8 <= len(buf) and buf[j:j + 2] == b"\x55\xcc" and buf[j + 3] == 0x57:
        ch = buf[j + 2]
        ln = struct.unpack_from("<I", buf, j + 4)[0]
        yield ch, buf[j + 8:j + 8 + ln]
        j += 8 + ln
        j += -j % WRAP_BLOCK


def unwrap(buf):
    """Yield DUML frames from one USB read."""
    for ch, body in blocks(buf):
        if ch == CH_DUML:
            yield body


def records(path):
    """Iterate (time, usb_read) from a .rec capture."""
    d = open(path, "rb").read()
    i = 0
    while i + 12 <= len(d):
        t, n = struct.unpack_from("<dI", d, i)
        yield t, d[i + 12:i + 12 + n]
        i += 12 + n


# RC-N1C stick report: RC (0x06) -> app, set 06 id 05, ~9 Hz, mapped 2026-11-02 (Mode 2).
STICK_MIN, STICK_CENTER, STICK_MAX = 364, 1024, 1684
FLIGHT_MODES = {0x2000: "C", 0x1000: "N", 0x0000: "S"}  # ch5 bits 12-13 (C = Cine, S = Sport)
BUTTONS_CH5 = {0x0800: "rth"}  # Flight Pause / RTH
BUTTONS_TAIL = {0x10: "fn", 0x01: "photo_video", 0x40: "shutter"}


def parse_sticks(payload):
    """Decode a set 06 id 05 payload (6 x u16 LE + 1 byte).

    Sticks and wheel are raw 364..1684, centre 1024. Up / right is the high end."""
    roll, pitch, throttle, yaw, wheel, sw = struct.unpack_from("<6H", payload)
    tail = payload[12] if len(payload) > 12 else 0
    buttons = [n for bit, n in BUTTONS_CH5.items() if sw & bit]
    buttons += [n for bit, n in BUTTONS_TAIL.items() if tail & bit]
    return dict(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw, wheel=wheel,
                mode=FLIGHT_MODES.get(sw & 0x3000, hex(sw & 0x3000)), buttons=buttons,
                raw_switches=sw, raw_tail=tail)


# Flight controller status ("OSD general"): FC (0x03) -> app, set 03 id 43, ~10 Hz.
# Layout from o-gs dji-firmware-tools comm_dissector (dji-dumlv1-flyc.lua).
FLYC_STATES = {0x00: "Manual", 0x01: "Atti", 0x03: "Atti_Hover", 0x04: "Hover", 0x06: "GPS_Atti",
               0x0a: "AssistedTakeoff", 0x0b: "AutoTakeoff", 0x0c: "AutoLanding", 0x0d: "AttiLanding",
               0x0f: "GoHome", 0x11: "Joystick", 0x17: "Atti_Limited", 0x18: "GPS_Atti_Limited",
               0x1f: "Sport", 0x21: "ForceLanding", 0x29: "EngineStart"}


def parse_osd_general(payload):
    """Decode a set 03 id 43 payload (55 B on the Mini 4K, relayed by the ground radio 0x0e).

    height/velocity/attitude match the o-gs layout and read sensibly on the ground.
    Flag word on the Mini 4K (props-off test 2026-11-03): 0x00805000 parked;
    +0x08 (motors on) and +0x02 within 1 s of the start gesture, +0x10 while
    idling/stopping; back to 0x00805000 after ~2 s of throttle down. In flight
    (2026-11-03) +0x04 (in air) and +0x100 (vision positioning in use), as in o-gs.
    State goes GPS_Atti -> EngineStart -> AssistedTakeoff -> Atti -> GPS_Atti."""
    lon, lat, h, vgx, vgy, vgz, pitch, roll, yaw = struct.unpack_from("<2d7h", payload)
    state, _cmd, flags = struct.unpack_from("<BBI", payload, 30)
    return dict(height_m=h / 10, vx=vgx / 10, vy=vgy / 10, vz=vgz / 10,
                pitch=pitch / 10, roll=roll / 10, yaw=yaw / 10,
                state=FLYC_STATES.get(state & 0x7F, hex(state & 0x7F)), flags=flags,
                motors_on=bool(flags & 0x08), in_air=bool(flags & 0x04),
                vision_positioning=bool(flags & 0x100))
