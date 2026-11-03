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
