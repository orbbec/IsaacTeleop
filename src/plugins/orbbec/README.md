<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Orbbec Ego Camera Plugin

This plugin connects capability-compatible Orbbec Ego stereo cameras to Isaac
Teleop. It captures `ColorLeft` and `ColorRight` video and, when the connected
device exposes them, accelerometer, gyroscope, microphone, calibration, and
device state.

The validated reference device is Ego PID `0x1201`, firmware `1.1.1`, on USB
2.0/480 Mbps with an Orbbec SDK reporting runtime version `20900`. PID is
reported, not used as the sole compatibility check. The plugin selects a
device from its actual left/right color sensors and advertised profiles. Ego
has no Depth or IR sensors, so depth, IR, D2C, and point-cloud features are
intentionally out of scope.

## What you can do

- Record stereo MJPEG, H.264, or H.265 elementary streams without a container.
- Record 48 kHz, mono, S16_LE PCM audio as WAV.
- Record frame metadata, IMU batches, audio offsets, calibration, and device
  state in MCAP.
- Choose raw-media-only, plugin-local MCAP, or TeleopSession MCAP workflows.
- Create a self-contained private `embedded` MCAP containing encoded video and
  PCM audio, then export it back to video files and WAV.
- Inspect device capabilities and apply validated exposure, gain, white-balance,
  image, bitrate, and IMU controls.
- Preview stereo video with the plugin's SDL window or use the independent
  GPU `camera_viz` source.

Capabilities have two levels. `--list-capabilities` reports profiles advertised
for each sensor; starting the pipeline validates the exact simultaneous stream
combination. An unsupported combination fails with its requested profiles and
SDK error instead of silently falling back.

Firmware 1.1.1 advertises H.264 and H.265 `1600x1200@60`, and short
dual-stream capture tests showed no sequence gaps. Sustained strict-decode
testing subsequently found corrupt encoded frames, both video-only and with
IMU/audio. This build therefore rejects resolved encoded profiles above 30 FPS
as advertised-but-uncertified; `fps=0` cannot bypass that check and no 30 FPS
fallback is selected. The supported default remains stereo H.264
`1600x1300@30`.

### Firmware 1.1.1 reference profiles

The audited Left and Right sensors advertised the same video profiles:

| Format | Resolution | FPS |
|---|---:|---:|
| H.264 / H.265 | 1600x1300 | 30 |
| H.264 / H.265 | 1600x1200 | 30, 60 |
| MJPEG | 1600x1300 | 30 |
| MJPEG | 1600x1200 | 30 |

The SDK also advertised YUYV `640x480@30/15/10`, but this plugin intentionally
does not expose YUYV. It did not advertise `1600x1300@60` or MJPEG at 60 FPS.
These values describe one audited device and firmware, not a replacement for
runtime discovery.

## Prerequisites

Use Ubuntu 22.04 x86_64 and an extracted Linux x86_64 OrbbecSDK package. The
SDK is an external dependency and is not distributed by this repository. Its
root directory must contain:

```text
include/libobsensor/ObSensor.hpp
lib/OrbbecSDKConfig.cmake
lib/libOrbbecSDK.so.2
lib/extensions/
```

The standard host tools are CMake **3.24 or newer**, a C++ compiler, Python,
`uv`, FFmpeg, and the Isaac Teleop build dependencies. Reading a TOML run
configuration requires Python 3.11 or newer; it uses the standard `tomllib`
module and adds no package dependency. A recording-only build does **not** need
SDL2 or FFmpeg development packages.

```bash
sudo apt update
sudo apt install -y build-essential cmake ccache clang-format-14 patchelf \
  pkg-config libudev-dev ffmpeg jq usbutils
```

The SDL preview is an optional build feature. Install its development packages
only when building with `--preview`:

```bash
sudo apt install -y libsdl2-dev libavcodec-dev libavutil-dev libswscale-dev
```

Install the udev rules provided by the **same SDK build** before using the
camera as a non-root user. For the tested SDK package this is normally:

```bash
sudo /path/to/OrbbecSDK/shared/install_udev_rules.sh
```

Reconnect the camera after installing rules. A desktop session is required for
`--preview`. The reference Ego reports `bcdUSB 2.00` / `480M` in `lsusb -t`,
which is normal for this device even on a USB 3.x host port.

Do not identify an SDK binary by semantic version alone: different SDK builds
can report the same version. The workflow records the executable and resolved
`libOrbbecSDK.so.2` paths and SHA-256 hashes alongside the firmware, serial,
UID, USB connection, and timestamp capability reported by the native plugin.

## Quick start: build, record, verify

The recommended entry point is `orbbec_ego.sh`. It never installs packages,
downloads SDKs, or runs `sudo`; `doctor` reports missing requirements and shows
the appropriate command instead.

Open a fresh terminal at the repository root:

```bash
cd /absolute/path/to/IsaacTeleop
export ORBBEC_SDK=/absolute/path/to/OrbbecSDK_v2_linux_x86_64

# 1. Inspect host dependencies, SDK layout, USB access, and optional preview libraries.
./src/plugins/orbbec/orbbec_ego.sh doctor --sdk-root "$ORBBEC_SDK"

# 2. Configure and build only the Orbbec plugin and embedded-media exporter.
./src/plugins/orbbec/orbbec_ego.sh build --sdk-root "$ORBBEC_SDK" --jobs 8

# Add --preview only after installing the optional SDL/FFmpeg development packages.
# ./src/plugins/orbbec/orbbec_ego.sh build --sdk-root "$ORBBEC_SDK" --preview --jobs 8

# 3. Save the software provenance and connected camera's advertised profiles.
./src/plugins/orbbec/orbbec_ego.sh capabilities | tee capabilities.txt

# 4. Record the supported default: stereo H.264 1600x1300@30, plus available
#    IMU/audio, calibration, and a local metadata MCAP.
./src/plugins/orbbec/orbbec_ego.sh record --duration 30
```

The final command creates a Git-ignored directory:

```text
recordings/orbbec_ego_<timestamp>/
├── raw/ColorLeft.h264
├── raw/ColorRight.h264
├── Audio.wav
├── calibration.json
├── metadata.mcap
├── capabilities.txt
├── requested_profile.json
├── verification_report.txt
└── logs/capture.log
```

`verification_report.txt` is created after the automatic post-recording
verification succeeds. A standalone `verify` command refreshes it; a failed
run writes `verification_report.failed.txt` instead so incomplete evidence is
not confused with an accepted report.

`requested_profile.json` is a machine-readable record of the requested
format, width, height, and FPS for both `ColorLeft` and `ColorRight`. It records
the request, while frame metadata and FFprobe describe what was actually
captured.

For an unlimited recording, omit `--duration` and stop with `Ctrl-C`. Wait for
the final statistics and the shell prompt; this finalizes WAV and MCAP output.

Verify the run before sharing it. Do not type angle-bracket placeholders such
as `<timestamp>` literally: replace them with the real directory name.

```bash
python3 -m pip install --user mcap  # only if verify asks for it

RUN="$(ls -dt recordings/orbbec_ego_*/ | head -n 1)"
./src/plugins/orbbec/orbbec_ego.sh verify "$RUN"
```

Success prints `Verification passed`, writes `verification_report.txt`, and
reports the delivery MCAP path and SHA-256 fingerprints for the MCAP and media.
Both H.264 streams decode completely with FFmpeg, and the MCAP has a Footer,
its required topics, and monotonic per-topic log times. It also rejects any remaining
`.partial`, rejects nonzero sequence-gap or queue-drop counters, confirms both
video profiles match, checks each FFprobe codec/resolution/frame rate against
`requested_profile.json`, matches each decoded count to that stream's MCAP
video topics, and checks raw device timestamps for strict monotonicity and the
declared frame cadence within each capture epoch. Left and Right may begin at
different parameterized IDRs, so unequal totals are reported but are not by
themselves a failure. When
IMU or audio was requested, it also validates nonempty contiguous batches or
chunks, the declared profile, raw-device cadence, and at least 90% coverage of
the video timeline. Audio sample and byte counts must agree, embedded PCM must
match its metadata timeline, and the WAV sidecar or export must be 48 kHz,
mono, S16_LE. Embedded verification always exports and decodes the MCAP payload;
if retained sidecars exist, their video bytes must match the embedded payload
exactly.
Recordings created before this manifest was introduced remain verifiable, but
`verify` warns that the requested-versus-recorded profile check was skipped.

### One-command script reference

```bash
./src/plugins/orbbec/orbbec_ego.sh doctor --sdk-root "$ORBBEC_SDK"
./src/plugins/orbbec/orbbec_ego.sh build --sdk-root "$ORBBEC_SDK" [--preview] [--jobs N] [--clean]
./src/plugins/orbbec/orbbec_ego.sh capabilities [--device-uid UID]
./src/plugins/orbbec/orbbec_ego.sh record [options] [-- PLUGIN_OPTIONS...]
./src/plugins/orbbec/orbbec_ego.sh verify RUN_DIRECTORY
./src/plugins/orbbec/orbbec_ego.sh export-media RUN_DIRECTORY [--output DIRECTORY]
```

Useful recording options:

```bash
# Select format/profile or output location.
./src/plugins/orbbec/orbbec_ego.sh record --duration 30 \
  --format h265 --width 1600 --height 1300 --fps 30 \
  --output recordings/demo_h265

# Inspect advertised profiles and the active certification policy. Firmware
# 1.1.1 advertises 1600x1200@60, but this build rejects it after resolution.
./src/plugins/orbbec/orbbec_ego.sh capabilities

# Disable optional sensors, select a device, or show the SDL preview (after a --preview build).
./src/plugins/orbbec/orbbec_ego.sh capabilities --device-uid DEVICE_UID
./src/plugins/orbbec/orbbec_ego.sh record --duration 30 \
  --device-uid DEVICE_UID --no-imu --preview

# Change the same-device recovery window, or use 0 to retain fail-fast behavior.
./src/plugins/orbbec/orbbec_ego.sh record --duration 120 \
  --reconnect-timeout 60 --reconnect-interval-ms 1000

# Pass advanced native controls unchanged after --.
./src/plugins/orbbec/orbbec_ego.sh record --duration 30 -- \
  --bitrate=8 --dynamic-bitrate=on
```

### Reusable TOML recording configuration

Use a TOML file for repeatable acceptance runs or deployment settings. Start
from [`orbbec_ego.example.toml`](orbbec_ego.example.toml), copy it outside the
source tree if desired, and run from the repository root:

```bash
cp src/plugins/orbbec/orbbec_ego.example.toml recordings/ego_run.toml
./src/plugins/orbbec/orbbec_ego.sh record \
  --config recordings/ego_run.toml
```

The `[record]` table supports:

| TOML key | Type | Meaning |
|---|---|---|
| `duration` | integer | Timed capture in seconds; omit for `Ctrl-C`. |
| `output` | string | New run directory. Relative paths use the command's working directory. |
| `format` | string | `mjpg`, `h264`, or `h265`. |
| `width`, `height`, `fps` | integer | Exact stereo profile requested for both color streams. |
| `device_uid` | string | UID reported by `capabilities`; omit to use the only connected compatible device. |
| `imu`, `audio` | boolean | Request or disable the optional sensor streams. |
| `preview` | boolean | Enable the SDL stereo preview. |
| `mcap_media` | string | `metadata-only` or `embedded`. |
| `keep_media_sidecars` | boolean | Retain video/WAV sidecars in embedded mode. |
| `reconnect_timeout` | integer | Same-device recovery window in seconds; zero disables recovery. |
| `reconnect_interval_ms` | integer | Re-enumeration interval in milliseconds. |
| `preset`, `plugin` | string | Select a build preset or an explicit plugin executable. |
| `plugin_options` | string array | Native controls such as `--bitrate=8` and `--exposure=1000`. |

Stereo `ColorLeft` and `ColorRight` remain mandatory in this validated wrapper
workflow; `imu` and `audio` select the optional data streams. Unknown keys,
wrong TOML types, missing files, and malformed TOML fail before the output
directory or camera is touched. Explicit CLI options after `--config` override
the corresponding file values, while options after `--` are appended after
`plugin_options`. For example:

```bash
# Reuse the file but override this run's duration, output, and audio choice.
./src/plugins/orbbec/orbbec_ego.sh record \
  --config recordings/ego_run.toml \
  --duration 120 --output recordings/ego_acceptance --no-audio
```

`record` automatically requests IMU and audio only if the device advertises
them. It never silently changes an unsupported requested profile to another
profile. Advertising Left and Right profiles separately does not prove that a
particular video/IMU/audio combination can run simultaneously; pipeline start
and the completed `verify` run are the combination check.

For `--duration`, the wrapper sends `SIGINT`, allows up to 30 seconds for
draining and finalization, and preserves the plugin's real exit status. A
shutdown, control-restore, or MCAP-close failure therefore cannot be mistaken
for an intentional timeout. The duration includes SDK startup and readiness;
use at least 30 seconds for an acceptance recording.

### Disconnect recovery

Recording waits up to 30 seconds by default for a disconnected camera to
reappear with the same serial number and USB VID/PID. A different Orbbec device
is never substituted. SDK 2.9.0 EGO UIDs contain the USB enumeration address
and can change after a physical reconnect, so UID is treated as an enumeration
instance and may differ between capture epochs.
The SDK pipelines, callbacks, exact stream profiles, calibration, and requested
controls are rebuilt before capture resumes. Auxiliary streams must remain
active throughout a five-second readiness window before a new epoch is
committed. Set `--reconnect-timeout 0` to disable recovery;
`--reconnect-interval-ms` controls re-enumeration frequency.

Every successful restart opens a new `capture_epoch` in video, IMU, audio,
calibration, and device-state records. Hardware sequence numbers and raw device
timestamps may restart only across that boundary; local common-clock time stays
monotonic. H.264/H.265 output discards post-reconnect prediction frames until a
new parameterized IDR arrives. The final statistics report the epoch, attempts,
and successful reconnects, and `verify` checks sequence and timestamp
continuity independently inside each epoch.

Queue loss, output/MCAP/SchemaPusher failure, changed firmware or profiles, and
control-restore failure remain fatal. If the same device does not return before
the timeout, the process exits nonzero and preserves the `.partial` MCAP. A
`Ctrl-C` while the device is absent also preserves the incomplete recording.
Video, IMU, and audio are each monitored after recovery; a stream that stops
for five seconds triggers another recovery instead of leaving a partial sensor
set marked healthy.

## Recording modes

There are three recording workflows. `--mcap-filename` and
`--collection-prefix` are mutually exclusive.

| Workflow | Use it when | Media location |
|---|---|---|
| Raw media only | You only need video and/or WAV. | `.mjpg`, `.h264`, `.h265`, `.wav` files. |
| Plugin-local MCAP | You need camera timing and sensor data in one local MCAP. | Default: sidecar video/WAV plus `metadata.mcap`. |
| TeleopSession MCAP | You need camera data alongside hand, head, controller, or other trackers. | DeviceIO writes the shared MCAP; raw media remains sidecar by default. |

### Raw media only

Use the native plugin directly when no MCAP is needed:

```bash
ORBBEC_PLUGIN="$PWD/build-orbbec-py3.11/src/plugins/orbbec/app/camera_plugin_orbbec"
RUN="recordings/raw_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN/raw"

timeout -s INT 30 "$ORBBEC_PLUGIN" \
  --add-stream=camera=ColorLeft,output="$RUN/raw/ColorLeft.h264",format=h264,width=1600,height=1300,fps=30 \
  --add-stream=camera=ColorRight,output="$RUN/raw/ColorRight.h264",format=h264,width=1600,height=1300,fps=30 \
  --audio-output="$RUN/Audio.wav"
```

The command ends normally after 30 seconds. Check `ColorLeft.h264`,
`ColorRight.h264`, and `Audio.wav` are non-empty, then use the media checks in
[Inspect and play recordings](#inspect-and-play-recordings).

### Local metadata MCAP (default script behavior)

The quick-start `record` command uses this mode. It writes the lightweight
structured topics below while leaving high-throughput video and WAV as regular
files:

```text
orbbec_metadata/ColorLeft, orbbec_metadata/ColorRight
orbbec_imu/Accel,           orbbec_imu/Gyro
orbbec_audio/Audio
orbbec_calibration/Calibration
orbbec_device/DeviceState
```

This is the recommended default because it is easy to inspect, avoids copying
media bytes into an MCAP, and keeps video playable through standard tools.
Before a future build enables a 60 FPS profile, certify this mode independently
from embedded MCAP; the two workflows have different queue and storage load.

### Self-contained private embedded MCAP

Use `embedded` only when one private Orbbec MCAP must contain video bytes, PCM
audio, metadata, IMU, calibration, and state. It does not transcode the data.
During recording the file has a `.partial` suffix and is renamed only after a
valid MCAP Footer is written.

```bash
RUN="recordings/embedded_$(date +%Y%m%d_%H%M%S)"
./src/plugins/orbbec/orbbec_ego.sh record --duration 30 \
  --output "$RUN" --mcap-media embedded
./src/plugins/orbbec/orbbec_ego.sh verify "$RUN"
```

In this mode no sidecar video/WAV is written unless you add
`--keep-media-sidecars`. The private media channels are:

```text
orbbec_media/ColorLeft
orbbec_media/ColorRight
orbbec_media/Audio
```

Export a completed embedded MCAP for ordinary playback:

```bash
./src/plugins/orbbec/orbbec_ego.sh export-media "$RUN"
```

When `verify --plugin PATH` is used, the script requires
`orbbec_mcap_export_media` from that plugin's installed directory or adjacent
build-tree `export_media` directory. It does not fall back to an exporter from
another preset or build tree.

The output is `ColorLeft.<format>`, `ColorRight.<format>`, and `Audio.wav`.
A remaining `metadata.mcap.partial` means the capture was interrupted or
failed; do not treat it as a deliverable.

Future 60 FPS certification must include every intended auxiliary stream.
Video-only success would not establish that video plus 1000 Hz IMU, audio, and
embedded MCAP can all stop and drain cleanly on the same SDK/firmware build;
the current build keeps the resolved-profile certification gate closed.

### TeleopSession: one MCAP with other trackers

For multi-device recording, create the five Orbbec Trackers together with your
existing hand/head/controller Trackers, and give them channels in the same
`McapRecordingConfig`:

```python
from isaacteleop import deviceio

prefix = "orbbec_ego"
trackers = [
    deviceio.FrameMetadataTrackerOrbbec(
        prefix,
        [deviceio.OrbbecCameraStream.ColorLeft, deviceio.OrbbecCameraStream.ColorRight],
    ),
    deviceio.OrbbecImuTracker(prefix),
    deviceio.OrbbecAudioTracker(prefix),
    deviceio.OrbbecCalibrationTracker(prefix),
    deviceio.OrbbecDeviceStateTracker(prefix),
]

# Add these trackers to the same TeleopSession and McapRecordingConfig as the
# configured hand, head, and controller trackers.
```

Use plugin arguments such as:

```text
--collection-prefix=orbbec_ego
--add-stream=camera=ColorLeft,output=<run>/raw/ColorLeft.h264,format=h264,width=1600,height=1300,fps=30
--add-stream=camera=ColorRight,output=<run>/raw/ColorRight.h264,format=h264,width=1600,height=1300,fps=30
--enable-imu
--audio-output=<run>/Audio.wav
```

This publishes structured Orbbec data through SchemaPusher; the TeleopSession
MCAP then contains Orbbec channels and any configured hand/head/controller
channels. It requires a running OpenXR runtime. Camera-only testing is covered
by `examples/oxr/python/test_orbbec_camera.py`; a physical XR setup is required
to accept a true multi-device session.

Periodic device-state publication uses the identity captured when the device
was opened. It does not perform a synchronous SDK identity query in the capture
drain loop; disconnect detection remains covered by the SDK removal callback
and per-stream liveness checks. This avoids periodically stalling every
SchemaPusher stream while retaining same-device recovery behavior.

A 60-second PICO 4 Ultra + Ego acceptance run after this change recorded 3,866
head samples and 3,866 samples from each controller. The shared MCAP contained
1,801 frames from each color camera with matching sequence range 31–1831, no
gaps or duplicates, and P95 sample-to-Tracker latency of 0.477/0.702 ms for
left/right video, 18.932/19.025 ms for accelerometer/gyroscope, and 18.765 ms
for audio. The retained sidecars strictly decoded as 1,802 H.264 frames per
camera. Evidence is under
`recordings/performance/orbbec_pico_teleop_optimized_20260904/` in the local
acceptance workspace.

To make that same final TeleopSession MCAP self-contained, configure an
embedded media fragment and use matching plugin arguments. The fragment is
written by the camera process and merged only after both the plugin and session
have closed cleanly:

```python
recording.embedded_media_filename = "<run>/orbbec_media_fragment.mcap"
```

```text
--collection-prefix=orbbec_ego
--mcap-media=embedded
--mcap-media-spool=<run>/orbbec_media_fragment.mcap
--add-stream=camera=ColorLeft,format=h264,width=1600,height=1300,fps=30
--add-stream=camera=ColorRight,format=h264,width=1600,height=1300,fps=30
--enable-imu
--enable-audio
```

Do not forcibly terminate the TeleopSession. A completed final MCAP has no
`.partial` sibling and can be checked with `export-media` or a standard MCAP
reader; preserve any fragment or `.partial` file for diagnosis instead of
publishing it.

TeleopSession embedded media is a separate acceptance target. The current build
does not permit encoded 60 FPS; do not remove that gate based only on
plugin-local MCAP or raw-sidecar success.

### Replay structured Orbbec data

Isaac Teleop already supplies the MCAP reader through `ReplaySession`; no
Orbbec-specific reader needs to be implemented. From an environment containing
the built or installed `isaacteleop` Python package, replay all five Orbbec
Tracker types headlessly with:

```bash
python examples/mcap_record_replay/python/replay_orbbec.py \
  "$RUN/metadata.mcap"
```

The example reads `ColorLeft`/`ColorRight`, `Accel`/`Gyro`, audio,
calibration, and device-state records, prints their capture epochs and key
fields, and exits nonzero if any expected channel is empty. Use
`--max-updates N` for a short inspection. This replays structured metadata and
sensor records; use FFmpeg or `export-media` to decode and view video payloads.

## Inspect and play recordings

Elementary video has no container index. Validate it first, then remux it into
MP4 for a desktop player:

```bash
# Replace RUN with a real recording directory.
ffmpeg -v error -f h264 -i "$RUN/raw/ColorLeft.h264" -f null -
ffmpeg -v error -f h264 -i "$RUN/raw/ColorRight.h264" -f null -
ffprobe -v error -show_entries stream=codec_name,sample_rate,channels \
  -of default=noprint_wrappers=1 "$RUN/Audio.wav"

ffmpeg -y -f h264 -framerate 30 -i "$RUN/raw/ColorLeft.h264" -c:v copy "$RUN/ColorLeft.mp4"
xdg-open "$RUN/ColorLeft.mp4"
xdg-open "$RUN/Audio.wav"
```

The plugin removes only recognized Orbbec timestamp-only SEI units, including
the legacy `ORBBEC,EGO_` form and firmware 1.1.1's `EGO..._[LR],timestamp_us=...`
form. Other SEI data is preserved. Do not work around a decode failure by
stripping all SEI; preserve the run and capture log for diagnosis.

For H.265 replace `h264` with `hevc`; for MJPEG use:

```bash
ffmpeg -f mjpeg -framerate 30 -i "$RUN/raw/ColorLeft.mjpg" \
  -c:v libx264 -pix_fmt yuv420p "$RUN/ColorLeft.mp4"
```

MCAP is structured recording data, not a generic video player. List its topics
with the standard reader:

```bash
python3 - "$RUN/metadata.mcap" <<'PY'
from collections import Counter
from pathlib import Path
import sys
from mcap.reader import make_reader

path = Path(sys.argv[1])
with path.open("rb") as stream:
    counts = Counter(channel.topic for _, channel, _ in make_reader(stream).iter_messages())
for topic in sorted(counts):
    print(f"{topic}: {counts[topic]}")
PY
```

## Device controls

Always inspect the connected device first. Property availability, range, and
step are firmware- and active-profile-specific:

```bash
./src/plugins/orbbec/orbbec_ego.sh capabilities | tee capabilities.txt
```

The capability listing is a discovery snapshot, not a guarantee that a range
is unchanged after another profile is activated. When controls are requested,
the plugin resolves the target profiles, establishes their range context, and
then validates permissions, type, range, step, and readback. It restores
changed properties only after capture pipelines have stopped unless
`--persist-controls` is explicitly supplied.

```bash
./src/plugins/orbbec/orbbec_ego.sh record --duration 20 -- \
  --set-property=OB_PROP_COLOR_AUTO_EXPOSURE_BOOL=0 \
  --exposure=1000 --gain=100 \
  --set-property=OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL=0 \
  --white-balance=5000 --sharpness=20 --saturation=128 --contrast=50 \
  --power-frequency=1
```

Supported friendly options are `--exposure`, `--gain`, `--white-balance`,
`--brightness`, `--sharpness`, `--saturation`, `--contrast`, and
`--power-frequency`. `--set-property=SDK_PROPERTY_NAME=VALUE` is the escape
hatch for other writable SDK properties. Ego has no OAK-equivalent `--quality`;
use `--bitrate` and `--dynamic-bitrate` instead.

Do not use `--persist-controls` for routine testing. It intentionally leaves
the selected values on the physical device.

## Preview and GPU stereo

Add `--preview` to a native plugin command or the one-command script to open a
side-by-side SDL preview while recording:

```bash
./src/plugins/orbbec/orbbec_ego.sh record --duration 30 --preview
```

The preview is a convenience CPU decode path and can drop stale display frames
without changing recording timestamps. It must not be used as an NVDEC latency
acceptance path.

For GPU-resident frames, use the independent `camera_viz` source:

```bash
examples/camera_viz/camera_viz.sh setup \
  --with-orbbec --orbbec-sdk-root "$ORBBEC_SDK"
examples/camera_viz/camera_viz.sh run \
  examples/camera_viz/configs/orbbec_ego.yaml --mode window
```

`camera_viz` and the recording plugin both open the physical camera; do not run
them against the same Ego at the same time. Desktop window mode normally shows
the left eye; use the configuration's side-by-side option for stereo debugging.
XR mode submits separate left/right textures.

## Native CLI reference

The script covers normal work. Use the executable directly only for custom
workflows:

```bash
ORBBEC_PLUGIN="$PWD/build-orbbec-py3.11/src/plugins/orbbec/app/camera_plugin_orbbec"
"$ORBBEC_PLUGIN" --help
"$ORBBEC_PLUGIN" --list-capabilities
```

Important options:

| Option | Meaning |
|---|---|
| `--add-stream=camera=...,output=...[,format=...,width=...,height=...,fps=...]` | Add one ColorLeft or ColorRight stream. Stream values override global values. |
| `--device-uid=UID` | Select one enumerated camera. |
| `--reconnect-timeout=N` | Wait N seconds for the same physical device after disconnect; 0 disables recovery. |
| `--reconnect-interval-ms=N` | Set the device re-enumeration interval; default 1000 ms. |
| `--enable-imu --imu-rate=400\|1000` | Capture accelerometer and gyroscope when available. |
| `--audio-output=PATH.wav` | Capture PCM audio as WAV. |
| `--calibration-output=PATH.json` | Export structured calibration and original SDK YAML. |
| `--mcap-filename=PATH` | Write a plugin-local MCAP. |
| `--collection-prefix=PREFIX` | Publish data for a TeleopSession. |
| `--mcap-media=metadata-only\|embedded` | Select sidecar or private self-contained local media handling. |
| `--keep-media-sidecars` | Also write raw video/WAV in embedded mode. |
| `--mcap-media-spool=PATH` | Required for embedded TeleopSession media merging. |

`--list-capabilities` reports SDK and firmware identity, serial and UID, USB
connection, global-timestamp support, per-sensor advertised profiles, and
property ranges. The wrapper adds plugin and resolved SDK-library hashes so a
recording can distinguish SDK builds that share a version number. Global
timestamp is not synthesized when the device reports it unsupported; recorded
frames retain host monotonic arrival time and raw device timestamp separately.

## Build, installation, and review checks

The script uses the equivalent CMake commands below. Use them when integrating
the plugin into an existing build or CI job:

```bash
cmake -S . -B build-orbbec-py3.11 \
  -DISAAC_TELEOP_PYTHON_VERSION=3.11 \
  -DBUILD_VIZ=OFF \
  -DBUILD_PLUGIN_ORBBEC_CAMERA=ON \
  -DORBBEC_SDK_ROOT="$ORBBEC_SDK"
cmake --build build-orbbec-py3.11 \
  --target camera_plugin_orbbec orbbec_mcap_export_media --parallel
```

After installation, `orbbec_ego.sh` is installed beside the Orbbec plugin and
supports `capabilities`, `record`, and `verify`. Its `build` command is only
for a source checkout.

The checked-in clean-room Dockerfile is an **optional developer/CI check** for
reproducible builds. It is not required for normal camera use, recording, or
TeleopSession. Docker validates source + separately supplied SDK; camera
access, udev, SDL, CUDA, and XR remain host-side tests.

```bash
git archive --format=tar HEAD | sudo docker build --pull --no-cache \
  --build-context orbbec_sdk="$ORBBEC_SDK" \
  -f src/plugins/orbbec/cleanroom/Dockerfile \
  -t isaacteleop-orbbec-cleanroom -
```

### Hardware-free software CI

`Test Orbbec Software` runs on the protected `self-hosted`, `linux`,
`orbbec-sdk` runner. The runner has a compatible SDK installed and receives
its root through the protected `ORBBEC_SDK_ROOT` Actions variable; it must not
have an EGO camera attached. The job configures Python 3.11 with
`BUILD_PLUGIN_ORBBEC_CAMERA=ON` and `BUILD_VIZ=OFF`, then runs every CTest
whose name contains `Orbbec`.

This covers C++/Python schema round trips, encoded-frame and MCAP writer
behavior, wrapper validation and verification failures, and an MCAP writer →
ReplaySession round trip for both camera streams, IMU, audio, calibration, and
device state across capture epochs. It deliberately does not automate physical
USB removal/reinsertion or long-duration capture; those remain hardware
acceptance tests.

Run the same software-only suite locally after building with the SDK:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  UV_CACHE_DIR=/tmp/isaacteleop-uv-cache \
  ctest --test-dir build-orbbec-py3.11 --output-on-failure --parallel \
  --no-tests=error -R '[Oo]rbbec'
```

Before a review or release:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  UV_CACHE_DIR=/tmp/isaacteleop-uv-cache \
  ctest --test-dir build-orbbec-py3.11 --output-on-failure -R orbbec

SKIP=check-copyright-year pre-commit run --all-files
git diff --check
```

Do not commit `recordings/`, `build/`, SDK extracts, `Log/`, or IDE files. The
repository packages neither the OrbbecSDK nor its license; a product release
must separately verify SDK redistribution rights, supported firmware, and udev
installation.

## Troubleshooting

| Symptom | Check |
|---|---|
| `doctor` reports no camera or no USB access | Install the selected SDK's udev rules, reconnect the camera, then run `doctor` and `capabilities` again. |
| Requested profile fails | Run `capabilities`; request one exact advertised profile per sensor. A profile may still fail as a simultaneous combination; the plugin does not fall back. |
| Recorded profile does not match `requested_profile.json` | Preserve the run and log. The requested profile was not delivered exactly; do not accept it as a silent substitute. |
| Embedded `verify --plugin PATH` cannot find an exporter | Build or install `orbbec_mcap_export_media` beside that plugin or in the same CMake build tree; do not mix artifacts from different builds. |
| `verify` reports no MCAP reader | Run `python3 -m pip install --user mcap`. |
| Video does not open in a desktop player | Validate/remux it with the FFmpeg commands above; elementary streams are not MP4 files. |
| `.partial` MCAP remains | The recording did not close cleanly. Preserve its log for diagnosis and do not publish it as a completed capture. |
| Capture waits after a disconnect | Reconnect the same physical device before `--reconnect-timeout` expires. The log reports any UID change, each attempt, and the resumed capture epoch. |
| Reconnect times out | Preserve the log and `.partial` MCAP for diagnosis, reconnect the camera, and start a new run. |
| An advertised 60 FPS profile is rejected as uncertified | This is intentional: sustained firmware 1.1.1 tests produced decoder-corrupt frames. Use an advertised 30 FPS profile until the complete certification suite passes on a newer device/firmware/SDK combination. |
| Preview is delayed | It is a CPU convenience preview. Close other GPU/camera applications or use `camera_viz` for the GPU path. |
| SchemaPusher exits immediately | A running OpenXR runtime is required; an old `cloudxr.env` file alone is insufficient. |
| SchemaPusher latency spikes at a regular multi-second interval | Confirm the plugin includes the nonblocking periodic device-state path; synchronous SDK identity polling in the capture drain loop can stall all streams. |
