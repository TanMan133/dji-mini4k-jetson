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
import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

Gst.init(None)

# DJI sends no IDR/CRA frames (gradual decoding refresh), so NVDEC must be told
# to start decoding without one: is-gdr-stream=true. {codec} is h264 or h265.
PIPELINE = (
    "filesrc location={fifo} ! {codec}parse ! "
    "nvv4l2decoder is-gdr-stream=true ! "
    "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
    "appsink name=sink drop=true max-buffers=1 sync=false"
)
# After (re)joining the stream, the bottom of the picture is green until the
# intra-refresh band has swept it once (~8 frames at 720p).
WARMUP_FRAMES = 15


def wait_codec(fifo, timeout=20):
    """The gadget writes the stream's codec next to the FIFO once it has seen its parameter sets."""
    t_end = time.time() + timeout
    while time.time() < t_end:
        try:
            with open(fifo + ".codec") as f:
                codec = f.read().strip()
            if codec in ("h264", "h265"):
                return codec
        except FileNotFoundError:
            pass
        time.sleep(0.2)
    raise SystemExit(f"no {fifo}.codec after {timeout} s: is aoa_gadget.py running with --video, drone linked?")


class GstCapture:
    """cv2.VideoCapture-like reader that pulls BGR frames straight from appsink.

    OpenCV's GStreamer backend queries the frame size while opening and, on this
    keyframe-less stream, often gives up before the decoder has produced a frame;
    pulling samples directly just waits for the first one."""

    def __init__(self, pipeline, first_frame_timeout=15.0):
        self.pipe = Gst.parse_launch(pipeline)
        self.sink = self.pipe.get_by_name("sink")
        self.pipe.set_state(Gst.State.PLAYING)
        self.timeout = first_frame_timeout

    def isOpened(self):
        return self.pipe is not None

    def read(self):
        sample = self.sink.emit("try-pull-sample", int(self.timeout * Gst.SECOND))
        self.timeout = 2.0  # after the first frame, 2 s without one means the stream stopped
        if sample is None:
            return False, None
        caps = sample.get_caps().get_structure(0)
        w, h = caps.get_value("width"), caps.get_value("height")
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return False, None
        try:
            frame = np.frombuffer(info.data, np.uint8, count=w * h * 3).reshape(h, w, 3).copy()
        finally:
            buf.unmap(info)
        return True, frame

    def release(self):
        if self.pipe is not None:
            self.pipe.set_state(Gst.State.NULL)
            self.pipe = None


def open_capture(fifo):
    return GstCapture(PIPELINE.format(fifo=fifo, codec=wait_codec(fifo)))


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

    cap = open_capture(args.fifo)
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
