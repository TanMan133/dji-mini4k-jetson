#!/usr/bin/env python3
"""Map the RC-N1C stick report (RC 0x06 -> app, set 06 id 05) from a capture.

Prints each u16 channel's range and the time-ordered movements, so a scripted
stick sequence can be matched to channels.

  python3 tools/stick_map.py capture/sticks/rc_to_phone.rec
"""
import struct
import sys

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0])
import duml  # noqa: E402

CENTER, DEAD = 1024, 40


def samples(path):
    for t, buf in duml.records(path):
        for f in duml.unwrap(buf):
            p = duml.parse(f)
            if p and p["src"] == 0x06 and p["set"] == 0x06 and p["id"] == 0x05:
                yield t, p["payload"]


def main():
    rows = list(samples(sys.argv[1]))
    if not rows:
        sys.exit("no stick reports in capture")
    t0 = rows[0][0]
    n = (len(rows[0][1])) // 2
    vals = [struct.unpack_from(f"<{n}H", pl) for _, pl in rows]
    tails = {pl[2 * n:].hex(" ") for _, pl in rows}
    print(f"{len(rows)} reports over {rows[-1][0] - t0:.1f} s, {n} u16 channels, trailing bytes seen: {sorted(tails)}")
    for c in range(n):
        col = [v[c] for v in vals]
        print(f"ch{c}: min {min(col):5d}  max {max(col):5d}  first {col[0]:5d}  distinct {len(set(col))}")

    # movements: contiguous runs where any channel leaves its resting value
    rest = vals[0]
    events, cur = [], None
    for (t, pl), v in zip(rows, vals):
        moved = [c for c in range(n) if abs(v[c] - rest[c]) > DEAD]
        if moved:
            if cur is None:
                cur = {"t0": t, "t1": t, "lo": list(v), "hi": list(v), "chs": set()}
            cur["t1"] = t
            cur["chs"].update(moved)
            cur["lo"] = [min(a, b) for a, b in zip(cur["lo"], v)]
            cur["hi"] = [max(a, b) for a, b in zip(cur["hi"], v)]
        elif cur is not None:
            events.append(cur)
            cur = None
    if cur:
        events.append(cur)
    print("\nmovements (time from start, channels away from rest, extreme reached):")
    for e in events:
        desc = []
        for c in sorted(e["chs"]):
            ext = e["hi"][c] if e["hi"][c] - rest[c] >= rest[c] - e["lo"][c] else e["lo"][c]
            desc.append(f"ch{c}->{ext}")
        print(f"  {e['t0'] - t0:6.1f}-{e['t1'] - t0:6.1f} s  {'  '.join(desc)}")


if __name__ == "__main__":
    main()
