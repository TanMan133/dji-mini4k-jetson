#!/usr/bin/env python3
"""Feed a recorded live view into a FIFO the way aoa_gadget.py --video does, looping,
so follow.py / live_view.py can be tested without the drone.

  .venv/bin/python tools/replay_video.py capture/vidprobe/rc_to_phone.rec capture/bench/video.fifo
"""
import os
import sys
import time

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0])
import duml  # noqa: E402


def main():
    rec, fifo = sys.argv[1], sys.argv[2]
    chunks = [(t, b) for t, r in duml.records(rec) for ch, b in duml.blocks(r) if ch == duml.CH_VIDEO]
    codec = "h265" if any(b"\x00\x00\x00\x01\x40\x01" in c for _, c in chunks[:400]) else "h264"
    sig = b"\x00\x00\x00\x01\x40\x01" if codec == "h265" else b"\x00\x00\x00\x01\x67"
    chunks = chunks[next(i for i, (_, c) in enumerate(chunks) if c.startswith(sig)):]
    t0 = chunks[0][0]
    if not os.path.exists(fifo):
        os.mkfifo(fifo)
    with open(fifo + ".codec", "w") as f:
        f.write(codec)
    while True:  # filesrc reopens the FIFO; start over from the parameter sets each time
        fd = os.open(fifo, os.O_WRONLY)
        try:
            while True:
                start = time.time()
                for t, c in chunks:
                    d = start + (t - t0) - time.time()
                    if d > 0:
                        time.sleep(d)
                    os.write(fd, c)
        except BrokenPipeError:
            os.close(fd)


if __name__ == "__main__":
    main()
