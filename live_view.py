#!/usr/bin/env python3
"""Pull decoded DJI live-view frames for inference.

The gadget (aoa_gadget.py --video FIFO) writes the drone's HEVC stream into a
named pipe; this decodes it on NVDEC and hands BGR numpy frames to infer().

  python3 live_view.py --fifo capture/live/video.fifo --seconds 30 --snap-every 5
"""
import argparse
import os
import time

import cv2

# DJI sends no IDR/CRA frames (gradual decoding refresh), so NVDEC must be told
# to start decoding without one: is-gdr-stream=true.
PIPELINE = (
    "filesrc location={fifo} ! h265parse ! "
    "nvv4l2decoder is-gdr-stream=true ! "
    "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)
# After (re)joining the stream, the bottom of the picture is green until the
# intra-refresh band has swept it once (~8 frames at 720p).
WARMUP_FRAMES = 15


def infer(frame):
    """Replace with the real model (TensorRT / ultralytics / ...)."""
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fifo", default="capture/live/video.fifo")
    ap.add_argument("--seconds", type=float, default=0, help="0 = run until Ctrl-C")
    ap.add_argument("--snap-every", type=float, default=0, help="save a JPEG every N s")
    ap.add_argument("--out", default="capture/live/snaps")
    args = ap.parse_args()

    cap = cv2.VideoCapture(PIPELINE.format(fifo=args.fifo), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit("could not open the GStreamer pipeline")
    os.makedirs(args.out, exist_ok=True)

    n, t0 = 0, time.time()
    t_stat, n_stat, t_snap = t0, 0, 0
    while not args.seconds or time.time() - t0 < args.seconds:
        ok, frame = cap.read()
        if not ok:
            print("stream ended")
            break
        n += 1
        if n <= WARMUP_FRAMES:
            continue
        infer(frame)
        now = time.time()
        if args.snap_every and now - t_snap >= args.snap_every:
            t_snap = now
            path = os.path.join(args.out, f"snap_{time.strftime('%H%M%S')}.jpg")
            cv2.imwrite(path, frame)
            print("saved", path)
        if now - t_stat >= 2:
            print(f"{frame.shape[1]}x{frame.shape[0]}  {(n - n_stat) / (now - t_stat):.1f} fps  ({n} frames)", flush=True)
            t_stat, n_stat = now, n
    cap.release()


if __name__ == "__main__":
    main()
