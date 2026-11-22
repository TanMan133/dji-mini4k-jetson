# DJI Mini 4K on a Jetson, through its own RC-N1C

A Jetson Orin Nano takes the phone's place on the DJI RC-N1C's cable: it receives the
drone's live video and telemetry over USB, runs person / hand detection on the GPU, and
flies the drone by driving the RC's stick signals from an ESP32. No DJI app or SDK is
involved, and nothing on the drone is modified.

What works (flown indoors, Mini 4K + RC-N1C, Jetson Orin Nano, ESP32-S3):

- live view into the Jetson (H.264 or H.265, 1280x720, 30 fps) and drone telemetry
  (height, attitude, flight state, motors, battery, gimbal angle)
- the Jetson commanding all four sticks and the camera wheel, within about 1 stick count
- take-off, hover and landing from the Jetson
- **person follow**: yaw keeps you centred, pitch holds your distance
- **hand control**: raise a hand and the drone turns to look at it, climbs and descends
  with it, and keeps a set distance; lower it and the drone holds

## Safety: read this first

This puts software in direct control of a flying aircraft. You are responsible for
flying it safely and legally where you are.

- **There is no stick override.** The ESP32 replaces the stick signals; the RC's own
  sticks do nothing while it is wired in. Your stops are: **SW2 on the ESP32 board
  (land)**, SW1 (hold), the LAND / HOVER buttons on the control page, and the drone's
  own behaviour when sticks centre (it holds position). If you want the sticks back
  as a hardware fallback, add the analog switches described under
  [Optional manual takeover](#optional-manual-takeover).
- **Test with the propellers off** until a new setup or code change has passed on the
  ground: the props-off motor test, the stick calibration and a motors-off dry run.
- **Power order matters.** Power the RC before the ESP32, and unplug the ESP32 before
  switching the RC off. A powered ESP32 back-feeds the stick lines of an unpowered RC,
  which then won't turn on.
- The motor-start gesture in the air is DJI's emergency motor stop, so `start` and
  `takeoff` are refused above 0.3 m; follow modes never move roll, so they can't form it.
- Indoors the drone holds position with its downward camera: use good light and a
  textured floor, keep clear space behind it (it backs up blind), and fly in N (Normal).
- Clear any DJI Fly take-off warning (e.g. error 30064) in the app first; the drone
  refuses to start its motors until then.

No warranty: see [LICENSE](LICENSE).

## How it works

```
drone ──O2 radio──► RC-N1C ──USB (AOA)──► Jetson USB-C  aoa_gadget.py  ──► video FIFO + telemetry UDP
                      ▲                                                          │
                      │ stick lines (5 x filtered PWM)                            ▼
                    ESP32-S3 ◄──USB serial── rc_control.py ◄── follow.py (YOLO11n / YOLO11n-pose on TensorRT)
```

- **USB link.** The RC is the USB host and speaks Android Open Accessory. `aoa_gadget.py`
  makes the Jetson's USB-C port (configfs + FunctionFS on `tegra-xudc`) look like a phone,
  answers the handshake, acknowledges the RC's messages, and splits the stream into DUML
  telemetry and live video.
- **Sticks.** Each stick axis and the camera wheel is a 3.3 V analog signal from a sensor
  board in the RC. Its wire to the main board is cut; the main-board end is fed from an
  ESP32 PWM pin through a two-stage 10 kΩ / 100 nF filter. `calibrate_sticks.py` maps PWM
  duty to the stick value the RC itself reports.

## Hardware

- DJI Mini 4K with RC-N1C (other O2 drones with the RC-N1 likely work; not tested)
- Jetson Orin Nano (JetPack with TensorRT; its USB-C port in device mode)
- ESP32-S3 (an ESP32-S3-MINI-1 board was used; 4 MB flash)
- 10 x 10 kΩ, 10 x 100 nF, thin wire

| ESP32 pin | RC signal (main-board end of the cut wire) |
|---|---|
| IO2 | right stick, left/right (roll) |
| IO4 | right stick, up/down (pitch) |
| IO6 | left stick, up/down (throttle)* |
| IO8 | left stick, left/right (yaw)* |
| IO10 | camera wheel signal |
| GND | RC ground (a stick module GND pad) |
| IO12 / IO13 | the board's SW1 (hold) / SW2 (land) buttons |

\*Which AD wire is which axis varies; the calibration detects the mapping and any
reversal, so a swapped pair needs no rewiring. Each filter: pin → 10 kΩ → 100 nF to GND →
10 kΩ → 100 nF to GND → RC. Leave the sensors' 3V3, GND and C1 wires connected, and never
join the ESP32's 3.3 V or 5 V to the RC.

### Optional manual takeover

A TS5A23157 analog switch per pair of channels (NC = stick sensor, NO = ESP32 filter,
COM = main board, select pulled low and driven by the ESP32 through a toggle) gives the
sticks back whenever the toggle is off or the ESP32 is unpowered.

## Setup

```
python3 -m venv --system-site-packages .venv          # system OpenCV (GStreamer) + TensorRT
.venv/bin/pip install cuda-python==13.3.1             # match your CUDA
```

Models (not included; Ultralytics YOLO11, AGPL-3.0), built into TensorRT engines on the Jetson:

```
python -c "from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='onnx', imgsz=640, opset=17, simplify=True)"
python -c "from ultralytics import YOLO; YOLO('yolo11n-pose.pt').export(format='onnx', imgsz=640, opset=17, simplify=True)"
/usr/src/tensorrt/bin/trtexec --onnx=yolo11n.onnx --saveEngine=models/yolo11n-fp16.engine --fp16
/usr/src/tensorrt/bin/trtexec --onnx=yolo11n-pose.onnx --saveEngine=models/yolo11n-pose-fp16.engine --fp16
```

ESP32 firmware (`firmware/esp32_sticks`, PlatformIO, Arduino-ESP32 3.x):

```
pio run
python -m esptool --chip esp32s3 --port /dev/ttyACM0 write-flash 0x0 .pio/build/esp32s3mini/firmware.factory.bin
```

## Use

```
# USB link: video + telemetry (as root; unbinds NVIDIA's l4t USB gadget while running)
sudo python3 aoa_gadget.py --ack --video capture/live/video.fifo --sticks-udp 127.0.0.1:9870 \
     --seconds 3600 --out capture/live

# once per RC wiring, drone off
.venv/bin/python tools/smoke_sticks.py      # each output moves one stick
.venv/bin/python calibrate_sticks.py        # writes calibration/sticks.json

# fly (restarts itself if the drone switches video codec)
./run_follow.sh --out capture/live                       # person follow (add --distance)
./run_follow.sh --out capture/live --target hand         # hand control
```

Commands go to `capture/live/follow.fifo` (one per line) or to the control page at
`http://<jetson>:8090/` (live view with detections, LAND / HOVER / FOLLOW / TAKE OFF):
`takeoff`, `follow`, `hover`, `land`, `level` (gimbal to 0°), `distcal` (you stand at
`--dist-m`: calibrates the hand-mode distance), `release` (after SW1/SW2), `idle`, `quit`.

Hand control: face the drone, hold one hand up beside your shoulder (wrist above elbow,
away from your face) for half a second. Left/right turns it, up/down raises or lowers it,
stepping closer/farther moves it back/forward (default 1.17 m). Lower your arm to let go.

Every run logs a per-frame CSV (detections, commands, telemetry, why a hand was or wasn't
used) and saves annotated frames at 10 fps (`--save-fps`) next to it; `aoa_gadget.py
--rec-video` also keeps the raw stream.

## Files

| | |
|---|---|
| `aoa_gadget.py` | USB gadget: AOA handshake, DUML acks, video FIFO, telemetry UDP, command FIFO |
| `duml.py` | DUML v1 + the RC's USB block framing; stick, FC status and battery decoders |
| `live_view.py` | NVDEC video reader (GStreamer appsink) |
| `detector.py` | YOLO11n / YOLO11n-pose on TensorRT |
| `rc_control.py` | ESP32 serial link, telemetry listener, calibrated stick commands |
| `follow.py`, `run_follow.sh` | person / hand follow, take-off and landing, control page |
| `calibrate_sticks.py` | PWM duty ↔ RC stick value tables |
| `firmware/esp32_sticks` | ESP32-S3 stick driver |
| `tools/` | stick mapping, smoke test, DUML stats, video latency and replay |
| `samples/` | a 9 s live-view clip, decoded frames, the AOA handshake |

## The USB link in detail

- AOA handshake: DJI / com.dji.logiclink / DJI RC-N1C / v04.11.00 → re-enumerate as
  18d1:2d00 (once in accessory mode the RC reconnects directly).
- Bulk OUT (RC→phone) is a stream of blocks: `55 cc <ch> 57` + le32 length + body, each
  zero-padded to a multiple of 512 B.
  - ch 0x49: one DUML v1 frame (CRC8 seed 0x77, CRC16 seed 0x3692).
  - ch 0x4a: live view, Annex B in chunks of ≤ 2040 B. H.265 Main or H.264 High,
    1280x720 ~30 fps, ~3 Mbit/s; the drone switched codec after pairing with DJI Fly on
    iOS and again mid-session, so the gadget detects it and writes `<fifo>.codec`.
    Each frame's last block ends with SEI + AUDs.
- Video flows as soon as the RC is paired with a powered drone; acking the RC's
  messages is all the "app" has to do.
- No IDR/CRA frames (gradual decoding refresh): NVDEC needs
  `nvv4l2decoder is-gdr-stream=true`; the first ~8 frames after joining are partly green.

### Messages used

| from → to | set / id | content |
|---|---|---|
| RC 06 → app | 06 / 05 | sticks, ~9 Hz: 6 x u16 + 1 byte (below) |
| RC 06 → app | 06 / 1e | RC battery, byte 4 = % |
| FC (via 0e) → app | 03 / 43 | FC status, 55 B: height (0.1 m, drifts indoors), velocities, attitude, state, flags 0x04 in air / 0x08 motors / 0x100 vision positioning |
| battery 0b → app | 0d / 02 | pack V, A, mAh, temperature, % |
| gimbal 04 → app | 04 / 05 | pitch / roll / yaw, 0.1° |

Stick report payload (decode with `duml.parse_sticks()`):

| field | stick (Mode 2) | low end | centre | high end |
|---|---|---|---|---|
| u16[0] | right stick left/right (roll) | 364 = left | 1024 | 1684 = right |
| u16[1] | right stick up/down (pitch) | 364 = down | 1024 | 1684 = up |
| u16[2] | left stick up/down (throttle) | 364 = down | 1024 | 1684 = up |
| u16[3] | left stick left/right (yaw) | 364 = left | 1024 | 1684 = right |
| u16[4] | camera wheel | 364 | 1024 | 1684 |
| u16[5] | flight mode bits 12-13: 0x2000 C, 0x1000 N, 0x0000 S; bit 11 (0x0800) = RTH button | | | |
| byte 12 | buttons: 0x10 Fn, 0x01 photo/video, 0x40 shutter | | | |

The RC has a dead zone around centre (about ±100 duty counts of 4096).

## Latency (measured)

| stage | |
|---|---|
| video arrives → decoded frame (640x360 on the GPU) | 33 ms H.264, 63 ms H.265 |
| frame → stick command | ~16 ms |
| command → RC registers it | ~40 ms |

DJI's O2 link and the drone's response add an estimated 0.2–0.4 s; `--lead-s` (prediction)
offsets part of it for moving targets.

## Gotchas

- Blocking FunctionFS reads deadlock a UDC unbind (process stuck in D state): ep0 is
  O_NONBLOCK + poll, and the bulk reader runs in a child process killed before unbind.
- GStreamer's `filesrc` opens the video FIFO, closes it and opens it again; the gadget
  re-attaches. A second NVDEC pipeline in the same process fails, hence `run_follow.sh`.
- OpenCV's GStreamer capture gives up on this keyframe-less stream; `live_view.GstCapture`
  pulls frames from appsink directly.
- ESP32-S3: the board's 4 MB flash needs `board_upload.flash_size = 4MB`; 12-bit PWM at
  19.5 kHz needs the 80 MHz APB clock; open its USB serial with DTR/RTS released (they
  reset the chip); flashing the factory image wipes its stored settings (`Sticks()`
  pushes them again on connect).

`samples/mini4k_liveview_9s.h265` decodes with:

    gst-launch-1.0 filesrc location=mini4k_liveview_9s.h265 ! h265parse ! nvv4l2decoder is-gdr-stream=true ! nvvidconv ! jpegenc ! multifilesink location=f_%04d.jpg

## Credits and licence

DUML framing and message layouts build on [o-gs/dji-firmware-tools](https://github.com/o-gs/dji-firmware-tools).
Not affiliated with or endorsed by DJI. Licensed under the [GNU AGPL-3.0](LICENSE).
