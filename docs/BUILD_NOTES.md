# Build notes

Facts needed to reproduce this setup, beyond the README. Terse on purpose: hand this file
and the README to an LLM and ask it about your step.

## Tested with

- DJI Mini 4K + RC-N1C (RC firmware v04.11.00); drone firmware not recorded
- Jetson Orin Nano, L4T R39.2.1, kernel 6.8 tegra, TensorRT 10.16, CUDA 13, Python 3.12
- ESP32-S3-MINI-1 board (UCSD EnVision "ESP32-S3-mini DevBoard", 4 MB flash),
  PlatformIO with the pioarduino espressif32 platform, Arduino-ESP32 3.3.x

## Before anything else

- Activate and pair the drone with the RC using DJI Fly (phone on the RC), update
  firmware there, and clear any take-off warning. Error 30064 ("unable to take off")
  blocked the motors until it was cleared in DJI Fly.
- The RC talks to the Jetson only on its phone cable (the USB-C lead stored in the RC),
  plugged into the Jetson's device-mode USB-C port (the one NVIDIA's `l4t` USB-network
  gadget uses; UDC `3550000.usb` on the Orin Nano dev kit). USB-A ports won't work: the
  RC is the USB host.
- The kernel needs USB gadget configfs with FunctionFS (`CONFIG_USB_CONFIGFS_F_FS`);
  stock JetPack has it.

## RC-N1C internals

- FCC ID SS3-RC2311911 (model RC231); internal photos at fccid.io/SS3-RC2311911.
- Each stick is a module with a small sensor board; its test pads are labelled
  `C1, AD1, AD0, 3V3, GND`. AD0/AD1 are the two axes as analog voltages (3.3 V supply).
  C1's function is unknown: leave it alone.
- The module harnesses plug into the main board through connectors covered in white
  glue. Cut the AD wires in the harness instead of freeing the connectors.
- The camera wheel is a separate small board (PP001406) with a 3-wire harness; its
  signal is analog like the sticks.
- Find which AD wire is which axis before cutting: probe each with the RC on while
  moving the stick, or watch the stick report (`tools/stick_map.py`). The calibration
  also detects swapped or reversed channels afterwards.
- The flight-mode switch and buttons were not modified and still work.
- The antennas are in the two ends of the phone clamp (TX/RX and RX), fed by thin coax.
- The battery appears (from the FCC photos, not measured) to be two 18650-size cells on
  a 2-pin connector.

## Wiring summary

Per channel: ESP32 pin → 10 kΩ → node A (100 nF to GND) → 10 kΩ → node B (100 nF to GND)
→ main-board end of the cut AD wire. Sensor end of the cut wire: insulate. One ground
wire from ESP32 GND to a stick module GND pad. Do not connect the ESP32's 3.3 V or 5 V to
the RC. Pin map in the README.

A single 10 kΩ + 1 µF stage also works with 11-bit PWM at 39 kHz (slower: ~45 ms
settling); the firmware is set for the two-stage filter.

## ESP32 notes

- Pins in the firmware (`firmware/esp32_sticks/src/main.cpp`): PWM on IO2/4/6/8/10,
  SW1 = IO12 (hold), SW2 = IO13 (land), LEDs IO17/IO18. Those buttons and LEDs are on
  the EnVision board; on another board, wire two buttons to IO12/IO13 (the firmware
  reads each button's resting level at boot, so either polarity works) or change the pins.
- Avoid IO0, IO3, IO45, IO46 (strapping), IO19/20 (USB), IO26 (module flash/PSRAM).
- PlatformIO's bundled esptool 5.4.0 crashed in its progress bar; flashing with
  `python -m esptool --silent ... write-flash 0x0 firmware.factory.bin` works.
  Flashing the factory image wipes the stored neutral/landing values (re-sent on connect).
- Open the S3's USB serial with DTR and RTS released, or it is held in reset. Opening the
  port still restarts it once.

## Python environments

- Runtime: `python3 -m venv --system-site-packages .venv`, then
  `.venv/bin/pip install -r requirements.txt`. System packages it relies on: OpenCV built
  with GStreamer (JetPack's), TensorRT's Python bindings, PyGObject (`python3-gi`), and
  GStreamer with NVIDIA's `nvv4l2decoder` / `nvvidconv`.
- Model export (any machine): a separate environment with `ultralytics` (pulls PyTorch)
  to export ONNX; build the TensorRT engines on the Jetson with `trtexec` (README).

## First run, in this order

1. `aoa_gadget.py` with the RC on: video and telemetry arrive, stick report reads centre.
2. Props off: `tools/smoke_sticks.py` (each output moves exactly one stick).
3. `calibrate_sticks.py` (drone off is fine): every channel reaches 364 / 1024 / 1684.
4. Props off, drone on: motor start and stop from the Jetson (`follow.py` command
   `start`, then `land`).
5. Props off: run follow / hand mode on the ground and check the commanded sticks move
   the right way.
6. Hand mode only: stand at `--dist-m` and send `distcal` once (shoulder widths vary;
   the 0.338 m default is one person's).
7. Fly: `tools/fly.py` (take-off, follow, lands on SW2/LAND, auto-lands at low battery).

## Quirks seen

- The drone sent H.265 at first and H.264 after pairing with DJI Fly on iOS, and once
  switched mid-session; `run_follow.sh` restarts the follow program when its video ends.
- The FC height reading drifts by up to ±1 m indoors; don't trust it for altitude.
- The RC's stick report arrives every ~110 ms; the RC reports a centre dead zone of about
  ±100 duty counts (of 4096).
- A floating (disconnected) stick input reads jittery values and copies other channels'
  extremes: check connectors if the smoke test shows cross-talk.
