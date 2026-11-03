#!/usr/bin/env python3
"""Summarise a rc_to_phone.rec capture: logiclink wrapper + DUML frames."""
import collections, struct, sys

def records(path):
    d = open(path, "rb").read()
    i = 0
    while i + 12 <= len(d):
        t, n = struct.unpack_from("<dI", d, i)
        yield t, d[i + 12:i + 12 + n]
        i += 12 + n

hdrs = collections.Counter(); cmds = collections.Counter(); samples = {}
t0 = t1 = None; nframes = 0; odd = 0
for t, buf in records(sys.argv[1]):
    t0 = t0 or t; t1 = t
    j = 0
    while j + 8 <= len(buf):
        magic, ln = buf[j:j + 4], struct.unpack_from("<I", buf, j + 4)[0]
        if magic != b"\x55\xcc\x49\x57":
            if any(buf[j:j + 8]): odd += 1; hdrs[buf[j:j+4].hex()] += 1
            break
        hdrs[magic.hex()] += 1
        f = buf[j + 8:j + 8 + ln]
        if f[:1] == b"\x55" and len(f) >= 13:
            src, dst, cmdset, cmdid = f[4], f[5], f[9], f[10]
            k = (src, dst, cmdset, cmdid)
            cmds[k] += 1; samples.setdefault(k, f[11:-2].hex(" "))
            nframes += 1
        j = (j + 8 + ln + 511) // 512 * 512 if ln + 8 <= 512 else j + 8 + ln
print(f"{nframes} DUML frames over {t1 - t0:.1f} s; wrapper magics {dict(hdrs)}; non-wrapper chunks {odd}")
for (s, d, cs, ci), n in cmds.most_common():
    print(f"{s:02x}->{d:02x} set {cs:02x} id {ci:02x}: {n:5d} ({n/(t1-t0):5.1f}/s)  {samples[(s,d,cs,ci)][:60]}")
