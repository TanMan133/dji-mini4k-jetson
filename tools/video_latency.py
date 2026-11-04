#!/usr/bin/env python3
"""Measure the video pipe -> decoded-frame latency offline, by replaying a capture.

Replays the live-view blocks of a capture recorded with `aoa_gadget.py --rec-video`
into a FIFO at their original timing, decodes them with live_view's pipeline, and
reports how long after a frame's last block was written the decoded frame
reached Python. Compare decoder settings with --decoder-opts. (DJI ends each
frame's last block with SEI + AUDs, so the parser sees frame ends at once.)

  .venv/bin/python tools/video_latency.py capture/vidprobe/rc_to_phone.rec
  .venv/bin/python tools/video_latency.py capture/vidprobe/rc_to_phone.rec --decoder-opts "disable-dpb=true"
"""
import argparse
import os
import sys
import tempfile
import threading
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0])
import duml  # noqa: E402
import live_view  # noqa: E402


def video_chunks(path):
    return [(t, body) for t, b in duml.records(path) for ch, body in duml.blocks(b) if ch == duml.CH_VIDEO]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--decoder-opts", default="", help="extra nvv4l2decoder properties")
    ap.add_argument("--seconds", type=float, default=12)
    ap.add_argument("--width", type=int, help="scale on the GPU, e.g. --width 640 --height 360")
    ap.add_argument("--height", type=int)
    args = ap.parse_args()

    chunks = video_chunks(args.rec)
    codec = "h265" if any(b"\x00\x00\x00\x01\x40\x01" in c for _, c in chunks[:400]) else "h264"
    sig = b"\x00\x00\x00\x01\x40\x01" if codec == "h265" else b"\x00\x00\x00\x01\x67"
    first = next(i for i, (_, c) in enumerate(chunks) if c.startswith(sig))
    chunks = chunks[first:]
    t0 = chunks[0][0]
    chunks = [(t - t0, c) for t, c in chunks if t - t0 < args.seconds]

    fifo = os.path.join(tempfile.mkdtemp(), "v.fifo")
    os.mkfifo(fifo)
    frame_done = []  # wall time when each frame's last block was written

    def writer():
        # filesrc opens the FIFO, closes it and opens it again (the gadget re-attaches the
        # same way), so start over from the parameter sets whenever the reader goes away
        while True:
            fd = os.open(fifo, os.O_WRONLY)
            frame_done.clear()
            start = time.time()
            try:
                for i, (t, c) in enumerate(chunks):
                    delay = start + t - time.time()
                    if delay > 0:
                        time.sleep(delay)
                    os.write(fd, c)
                    nxt = chunks[i + 1][1] if i + 1 < len(chunks) else b"\x00\x00\x00\x01"
                    if nxt.startswith(b"\x00\x00\x00\x01"):
                        frame_done.append(time.time())
            except BrokenPipeError:
                os.close(fd)
                continue
            time.sleep(1.0)
            os.close(fd)
            return

    scale = f",width={args.width},height={args.height}" if args.width else ""
    pipe = live_view.PIPELINE.format(fifo=fifo, codec=codec, scale=scale)
    if args.decoder_opts:
        pipe = pipe.replace("is-gdr-stream=true", "is-gdr-stream=true " + args.decoder_opts)
    threading.Thread(target=writer, daemon=True).start()
    cap = live_view.GstCapture(pipe)
    got = []
    while True:
        ok, _ = cap.read()
        if not ok:
            break
        got.append(time.time())
    cap.release()
    got = [g for g in got if not frame_done or g >= frame_done[0]]

    # decoded frame k should match written frame k + skip (the decoder may drop a few at the start)
    best = None
    for skip in range(0, 6):
        n = min(len(got), len(frame_done) - skip)
        if n < 30:
            continue
        lat = np.array(got[:n]) - np.array(frame_done[skip:skip + n])
        if lat.min() > -0.005 and (best is None or lat.std() < best[1].std()):
            best = (skip, lat)
    if best is None:
        sys.exit(f"could not align: {len(got)} decoded, {len(frame_done)} written")
    lat = best[1][10:] * 1000  # skip the start-up frames
    print(f"{codec} {args.decoder_opts or 'default decoder'}: "
          f"{len(got)} frames; latency median {np.median(lat):.1f} ms, p90 {np.percentile(lat, 90):.1f} ms, "
          f"max {lat.max():.1f} ms")


if __name__ == "__main__":
    main()
    os._exit(0)  # don't wait on GStreamer/CUDA teardown
