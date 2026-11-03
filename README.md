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

## Gotchas

- Blocking FunctionFS reads deadlock a UDC unbind (process stuck in D state): ep0 is
  O_NONBLOCK + poll, and the bulk reader runs in a child process that is SIGKILLed before unbind.
- This ffmpeg build has no software HEVC decoder; `hevc_nvv4l2dec` hangs on GDR streams. Use GStreamer.

samples/mini4k_liveview_9s.h265 decodes with:

    gst-launch-1.0 filesrc location=mini4k_liveview_9s.h265 ! h265parse ! nvv4l2decoder is-gdr-stream=true ! nvvidconv ! jpegenc ! multifilesink location=f_%04d.jpg
