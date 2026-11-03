# DJI Mini 4K live view on a Jetson via the RC-N1C USB link

The Jetson's USB-C port (tegra-xudc, UDC `3550000.usb`) pretends to be the phone.
The RC-N1C is the USB host and speaks Android Open Accessory (AOA).

## Run (on the Jetson, RC plugged into the Jetson USB-C port, drone paired + on)

    sudo python3 aoa_gadget.py --ack --video capture/live/video.fifo --seconds 600 --out capture/live
    python3 live_view.py --fifo capture/live/video.fifo --snap-every 10

`aoa_gadget.py` unbinds NVIDIA's l4t USB-network gadget while it runs and rebinds it on exit.
Captures go to `capture/` (git-ignored; raw `.rec` files grow ~25 MB/min with `--rec-video`).
Inject DUML commands into a running session: `echo "06 06 2f 40 01" > capture/live/cmd.fifo`
(dst set id type payload_hex; that one is "RC Pairing").

## What the link looks like

- AOA handshake: DJI / com.dji.logiclink / DJI RC-N1C / v04.11.00 → re-enumerate as 18d1:2d00.
- Bulk OUT (RC→phone) is a stream of blocks: `55 cc <ch> 57` + le32 length + body,
  each block zero-padded to a multiple of 512 B.
  - ch 0x49: one DUML v1 frame (CRC8 seed 0x77, CRC16 seed 0x3692) — telemetry, RC sticks (set 06 id 05).
  - ch 0x4a: live view, HEVC Main Annex B, 1280x720 ~30 fps, ~3 Mbit/s, chunks of ≤2040 B.
- Video flows as soon as the RC is paired to a powered drone; no app request needed.
- No IDR/CRA frames: gradual decoding refresh. NVDEC needs `nvv4l2decoder is-gdr-stream=true`;
  the first ~8 frames after joining show a green, not-yet-refreshed area (live_view.py skips 15).

## RC stick report (set 06 id 05, RC -> app, ~9 Hz)

Payload = 6 x u16 LE + 1 byte. Decode with `duml.parse_sticks()`; mapped with `tools/stick_map.py`.

| field | stick (Mode 2) | low end | centre | high end |
|---|---|---|---|---|
| u16[0] | right stick left/right (roll) | 364 = left | 1024 | 1684 = right |
| u16[1] | right stick up/down (pitch) | 364 = down | 1024 | 1684 = up |
| u16[2] | left stick up/down (throttle) | 364 = down | 1024 | 1684 = up |
| u16[3] | left stick left/right (yaw) | 364 = left | 1024 | 1684 = right |
| u16[4] | camera wheel | 388 seen | 1024 | 1673 seen |
| u16[5] | flight mode bits 12-13: 0x2000 C, 0x1000 N, 0x0000 S; bit 11 (0x0800) = RTH button | | | |
| byte 12 | buttons: 0x10 Fn, 0x01 photo/video, 0x40 shutter | | | |

Short button presses can fall between two reports (~110 ms apart).
RC battery: set 06 id 1e, bytes 4 = percent.

## Gotchas

- Blocking FunctionFS reads deadlock a UDC unbind (process stuck in D state): ep0 is
  O_NONBLOCK + poll, and the bulk reader runs in a child process that is SIGKILLed before unbind.
- This ffmpeg build has no software HEVC decoder; `hevc_nvv4l2dec` hangs on GDR streams. Use GStreamer.

samples/mini4k_liveview_9s.h265 decodes with:

    gst-launch-1.0 filesrc location=mini4k_liveview_9s.h265 ! h265parse ! nvv4l2decoder is-gdr-stream=true ! nvvidconv ! jpegenc ! multifilesink location=f_%04d.jpg
