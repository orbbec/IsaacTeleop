<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# EGO Camera Plugin

This plugin connects EGO stereo cameras to Isaac Capture. It captures
`ColorLeft` and `ColorRight` video and, when available, accelerometer,
gyroscope, microphone, calibration, and device state.

The plugin selects a device by its left/right color sensors and advertised
profiles. Run `ego.sh capabilities` for the connected device's firmware, USB
connection, and profiles. This plugin does not provide depth, IR, D2C,
point-cloud, or YUYV streams.

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

The wrapper defaults to stereo H.264 `1600x1300@30`. The plugin rejects
resolved H.264/H.265 profiles above 30 FPS, even if the device advertises them.
`fps=0` cannot bypass this check, and no lower-FPS fallback is selected.

## Prerequisites

For a Linux x86_64 host, use an extracted OrbbecSDK v2 package for the same
architecture and the normal Isaac Capture build dependencies. The SDK is
external and is not distributed by this repository. Its root must contain:

```text
include/libobsensor/ObSensor.hpp
lib/OrbbecSDKConfig.cmake
lib/libOrbbecSDK.so.2
lib/extensions/
```

The standard host tools are CMake **3.24 or newer**, a C++ compiler, Python,
`uv`, FFmpeg, and the Isaac Capture build dependencies. Reading a TOML run
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
camera as a non-root user. Use the script supplied with that package:

```bash
sudo /path/to/OrbbecSDK/shared/install_udev_rules.sh
```

Reconnect the camera after installing rules. A desktop session is required for
`--preview`.

Do not identify an SDK binary by semantic version alone: different SDK builds
can report the same version. The workflow records the executable and resolved
`libOrbbecSDK.so.2` paths and SHA-256 hashes alongside the firmware, serial,
UID, USB connection, and timestamp capability reported by the native plugin.

## Quick start: build, record, verify

The recommended entry point is `ego.sh`. It never installs packages,
downloads SDKs, or runs `sudo`; `doctor` reports missing requirements and shows
the appropriate command instead.

Open a fresh terminal at the repository root:

```bash
cd /absolute/path/to/IsaacCapture
export ORBBEC_SDK=/absolute/path/to/OrbbecSDK_v2_linux_x86_64

# 1. Inspect host dependencies, SDK layout, USB access, and optional preview libraries.
./src/plugins/ego/ego.sh doctor --sdk-root "$ORBBEC_SDK"

# 2. Configure and build only the Ego plugin and embedded-media exporter.
./src/plugins/ego/ego.sh build --sdk-root "$ORBBEC_SDK" --jobs 8

# Add --preview only after installing the optional SDL/FFmpeg development packages.
# ./src/plugins/ego/ego.sh build --sdk-root "$ORBBEC_SDK" --preview --jobs 8

# 3. Save the software provenance and connected camera's advertised profiles.
./src/plugins/ego/ego.sh capabilities | tee capabilities.txt

# 4. Record the supported default: stereo H.264 1600x1300@30, plus available
#    IMU/audio, calibration, and a local metadata MCAP.
./src/plugins/ego/ego.sh record --duration 30
```

The final command creates a Git-ignored directory:

```text
recordings/ego_<timestamp>/
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

RUN="$(ls -dt recordings/ego_*/ | head -n 1)"
./src/plugins/ego/ego.sh verify "$RUN"
```

Success prints `Verification passed`, writes `verification_report.txt`, and
reports the delivery MCAP path and SHA-256 fingerprints for the MCAP and media.
Both H.264 streams decode completely with FFmpeg, and the MCAP has a Footer,
its required topics, and monotonic per-topic log times. It also rejects any remaining
`.partial`, rejects nonzero sequence-gap or queue-drop counters, confirms both
video profiles match, checks each FFprobe codec/resolution against
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
If `requested_profile.json` is absent, `verify` warns that the
requested-versus-recorded profile check was skipped.

### One-command script reference

```bash
./src/plugins/ego/ego.sh doctor --sdk-root "$ORBBEC_SDK"
./src/plugins/ego/ego.sh build --sdk-root "$ORBBEC_SDK" [--preview] [--jobs N] [--clean]
./src/plugins/ego/ego.sh capabilities [--device-uid UID]
./src/plugins/ego/ego.sh record [options] [-- PLUGIN_OPTIONS...]
./src/plugins/ego/ego.sh verify RUN_DIRECTORY
./src/plugins/ego/ego.sh export-media RUN_DIRECTORY [--output DIRECTORY]
```

Useful recording options:

```bash
# Select format/profile or output location.
./src/plugins/ego/ego.sh record --duration 30 \
  --format h265 --width 1600 --height 1300 --fps 30 \
  --output recordings/demo_h265

# Inspect the connected device's advertised profiles.
./src/plugins/ego/ego.sh capabilities

# Disable optional sensors, select a device, or show the SDL preview (after a --preview build).
./src/plugins/ego/ego.sh capabilities --device-uid DEVICE_UID
./src/plugins/ego/ego.sh record --duration 30 \
  --device-uid DEVICE_UID --no-imu --preview

# Change the same-device recovery window, or use 0 to retain fail-fast behavior.
./src/plugins/ego/ego.sh record --duration 120 \
  --reconnect-timeout 60 --reconnect-interval-ms 1000

# Pass advanced native controls unchanged after --.
./src/plugins/ego/ego.sh record --duration 30 -- \
  --bitrate=8 --dynamic-bitrate=on
```

### Reusable TOML recording configuration

Use a TOML file for repeatable recordings or deployment settings. Start
from [`ego.example.toml`](ego.example.toml), copy it outside the
source tree if desired, and run from the repository root:

```bash
cp src/plugins/ego/ego.example.toml recordings/ego_run.toml
./src/plugins/ego/ego.sh record \
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

Stereo `ColorLeft` and `ColorRight` are mandatory in the wrapper
workflow; `imu` and `audio` select the optional data streams. Unknown keys,
wrong TOML types, missing files, and malformed TOML fail before the output
directory or camera is touched. Explicit CLI options after `--config` override
the corresponding file values, while options after `--` are appended after
`plugin_options`. For example:

```bash
# Reuse the file but override this run's duration, output, and audio choice.
./src/plugins/ego/ego.sh record \
  --config recordings/ego_run.toml \
  --duration 120 --output recordings/ego_run --no-audio
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
allow enough time for both before evaluating the recorded interval.

### Disconnect recovery

Recording waits up to 30 seconds by default for a disconnected camera to
reappear with the same serial number and USB VID/PID. A different Ego device
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
EGO_PLUGIN="$PWD/build-ego-py3.11/src/plugins/ego_camera/ego_camera_plugin"
RUN="recordings/raw_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN/raw"

timeout -s INT 30 "$EGO_PLUGIN" \
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
ego_metadata/ColorLeft, ego_metadata/ColorRight
ego_imu/Accel,           ego_imu/Gyro
ego_audio/Audio
ego_calibration/Calibration
ego_device/DeviceState
```

This mode avoids copying media bytes into an MCAP and keeps video playable
through standard tools.

### Self-contained private embedded MCAP

Use `embedded` only when one private Ego MCAP must contain video bytes, PCM
audio, metadata, IMU, calibration, and state. It does not transcode the data.
During recording the file has a `.partial` suffix and is renamed only after a
valid MCAP Footer is written and all capture, queue drain, media close, control
restoration, and SchemaPusher cleanup steps succeed. A Footer alone does not
prove a successful capture; cleanup failures retain the `.partial` file.

```bash
RUN="recordings/embedded_$(date +%Y%m%d_%H%M%S)"
./src/plugins/ego/ego.sh record --duration 30 \
  --output "$RUN" --mcap-media embedded
./src/plugins/ego/ego.sh verify "$RUN"
```

In this mode no sidecar video/WAV is written unless you add
`--keep-media-sidecars`. The private media channels are:

```text
ego_media/ColorLeft
ego_media/ColorRight
ego_media/Audio
```

Export a completed embedded MCAP for ordinary playback:

```bash
./src/plugins/ego/ego.sh export-media "$RUN"
```

When `verify --plugin PATH` is used, the script requires
`ego_mcap_export_media` from that plugin's installed directory or adjacent
build-tree `export_media` directory. It does not fall back to an exporter from
another preset or build tree.

The output is `ColorLeft.<format>`, `ColorRight.<format>`, and `Audio.wav`.
A remaining `metadata.mcap.partial` means the capture was interrupted or
failed; do not treat it as a deliverable.

### TeleopSession: one MCAP with other trackers

Complete the normal full build and install its Python wheel
before using the EGO trackers or `isaaccapture.ego` helper. The wrapper's
`build` command builds the native plugin tools only:

```bash
cmake --build build-ego-py3.11 --parallel
cmake --install build-ego-py3.11 --prefix install-ego-py3.11
uv pip install --reinstall install-ego-py3.11/wheels/isaaccapture-*.whl
```

For multi-device recording, create the five Ego Trackers together with your
existing hand/head/controller Trackers, and give them channels in the same
`McapRecordingConfig`:

```python
from isaaccapture.deviceio_trackers import (
    EgoFrameMetadataTracker, EgoImuTracker, EgoAudioTracker,
    EgoCalibrationTracker, EgoDeviceStateTracker,
)
from isaaccapture.deviceio_session import McapRecordingConfig
from isaaccapture.schema import EgoCameraStream

prefix = "ego"
trackers = [
    EgoFrameMetadataTracker(
        prefix,
        [EgoCameraStream.ColorLeft, EgoCameraStream.ColorRight],
    ),
    EgoImuTracker(prefix),
    EgoAudioTracker(prefix + "/Audio"),
    EgoCalibrationTracker(prefix + "/Calibration"),
    EgoDeviceStateTracker(prefix + "/DeviceState"),
]
tracker_names = list(zip(trackers, [
    "ego_metadata", "ego_imu", "ego_audio", "ego_calibration", "ego_device",
]))

# Add these trackers to the same TeleopSession and McapRecordingConfig as the
# configured hand, head, and controller trackers.
```

When a pipeline supplies these trackers through DeviceIO sources, use the same
`ego_metadata`, `ego_imu`, `ego_audio`, `ego_calibration`, and `ego_device` source
names. Source names determine MCAP topic bases; `--collection-prefix` determines
OpenXR collection IDs independently.

Use plugin arguments such as:

```text
--collection-prefix=ego
--add-stream=camera=ColorLeft,output=<run>/raw/ColorLeft.h264,format=h264,width=1600,height=1300,fps=30
--add-stream=camera=ColorRight,output=<run>/raw/ColorRight.h264,format=h264,width=1600,height=1300,fps=30
--enable-imu
--audio-output=<run>/Audio.wav
```

Set `shutdown_timeout_seconds=15.0` on this EGO `PluginConfig` to allow SDK stop,
queue drain, and control restoration before the process deadline. Other plugins
retain the default of two seconds.

This publishes structured Ego data through SchemaPusher; the TeleopSession
MCAP then contains Ego channels and any configured hand/head/controller
channels. It requires a running OpenXR runtime. See
`examples/oxr/python/test_ego_camera.py` for a camera-only session example.

Periodic device-state publication uses the identity captured when the device
was opened. It does not perform a synchronous SDK identity query in the capture
drain loop; disconnect detection remains covered by the SDK removal callback
and per-stream liveness checks. This avoids periodically stalling every
SchemaPusher stream while retaining same-device recovery behavior.

For a self-contained multi-device recording, use the EGO-only finalization
context around the ordinary `TeleopSessionConfig`. The existing pipeline and
hand/head/controller trackers stay in that config. Add exactly one enabled
`ego_camera` plugin with these arguments (absolute paths avoid the plugin
working-directory ambiguity):

```text
--collection-prefix=ego
--mcap-media=embedded
--mcap-media-spool=/absolute/run/ego_fragment.mcap
--add-stream=camera=ColorLeft,format=h264,width=1600,height=1300,fps=30
--add-stream=camera=ColorRight,format=h264,width=1600,height=1300,fps=30
--enable-imu
--enable-audio
```

```python
from pathlib import Path
from isaaccapture.ego import embedded_recording

run = Path("recordings/teleop_ego").resolve()
run.mkdir(parents=True, exist_ok=True)
# config is your live TeleopSessionConfig, with trackers/plugin args above.
config.mcap_config = McapRecordingConfig(
    str(run / "final.mcap"),
    tracker_names,
)
with embedded_recording(config, media_fragment=run / "ego_fragment.mcap") as session:
    try:
        while True:
            session.step()
    except KeyboardInterrupt:
        pass  # Exit the context normally to stop and finalize the recording.
```

The final name comes from `config.mcap_config.filename`. The context records
host tracker observations to `final.session.mcap`; the plugin writes a complete
structured-data plus media fragment. Only after the session and plugin
have stopped successfully does the adjacent `ego_mcap_merge` tool create
`final.mcap`. Canonical EGO topics come from the fragment, so the last captured
records do not depend on the host's final `session.step()`; overlapping host
observations use `session_observed/` topics in the merged file.

The final, session, and fragment paths must differ, their parent directories
must exist, and none of them or their `.partial` siblings may already exist.
`media_fragment` must match the plugin's `--mcap-media-spool`, resolved against
its working directory. Build/install `ego_mcap_merge` beside the plugin
before starting. A body exception, failed cleanup, or failed merge propagates
and leaves inputs or `.partial` output for diagnosis. Catch Ctrl+C inside the
context when you intend a clean stop; do not force-kill it.

Camera and IMU `get_stream_data(session, index)` return a payload or `None`.
Audio, calibration, and state use `get_data(session)` with the same contract.
Their collection IDs are the complete `prefix/Audio`, `prefix/Calibration`, and
`prefix/DeviceState` strings; payloads have no `.data` wrapper.

### Replay structured Ego data

Recordings use `Ego` schema types and `ego` topics; former Orbbec development
interfaces and recordings are unsupported.

From an environment containing the built or installed `isaaccapture` Python
package, replay all five EGO tracker types headlessly through `ReplaySession`:

```bash
uv pip install -e ./examples/mcap_record_replay
python -m isaaccapture_examples.mcap_record_replay.replay_ego \
  "$RUN/metadata.mcap"
```

The example reads `ColorLeft`/`ColorRight`, `Accel`/`Gyro`, audio,
calibration, and device-state records, prints their capture epochs and key
fields, and exits nonzero if any expected channel is empty. Use
`--max-updates N` for a short inspection. This replays structured metadata and
sensor records; use FFmpeg or `export-media` to decode and view video payloads.
The final MCAP produced by `isaaccapture.ego.embedded_recording` can also be
passed to this command.

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

The plugin removes only recognized Ego timestamp-only SEI units, including
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
./src/plugins/ego/ego.sh capabilities | tee capabilities.txt
```

The capability listing is a discovery snapshot, not a guarantee that a range
is unchanged after another profile is activated. When controls are requested,
the plugin resolves the target profiles, establishes their range context, and
then validates permissions, type, range, step, and readback. It restores
changed properties only after capture pipelines have stopped unless
`--persist-controls` is explicitly supplied.

```bash
./src/plugins/ego/ego.sh record --duration 20 -- \
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
./src/plugins/ego/ego.sh record --duration 30 --preview
```

The preview decodes on the CPU and can drop stale display frames without
changing recording timestamps.

For GPU-resident frames, use the independent `camera_viz` source:

```bash
examples/camera_viz/camera_viz.sh setup \
  --with-ego --sdk-root "$ORBBEC_SDK"
examples/camera_viz/camera_viz.sh run \
  examples/camera_viz/configs/ego.yaml --mode window
```

The config defaults to H.264 `1600x1300@30`. Set `format` to `h265` or `mjpg`
and choose an advertised stereo profile. H.264/H.265 requires a working CUDA
toolkit and the native NVDEC codec; MJPEG uses SDK RGB conversion and CuPy for
GPU upload.
The SDK's udev rules must be installed before opening the camera as a non-root
user. See the [EGO preview binding guide](../../../examples/camera_viz/ego_preview/README.md)
for binding builds, API, and GPU data flow.

`camera_viz` and the recording plugin both open the physical camera; do not run
them against the same EGO at the same time. For desktop stereo inspection and
XR output options, see the [viewer guide](../../../examples/camera_viz/README.md#mode-1--direct).

## Native CLI reference

The script covers normal work. Use the executable directly only for custom
workflows:

```bash
EGO_PLUGIN="$PWD/build-ego-py3.11/src/plugins/ego_camera/ego_camera_plugin"
"$EGO_PLUGIN" --help
"$EGO_PLUGIN" --list-capabilities
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
cmake -S . -B build-ego-py3.11 \
  -DISAAC_TELEOP_PYTHON_VERSION=3.11 \
  -DBUILD_VIZ=OFF \
  -DBUILD_PLUGIN_EGO_CAMERA=ON \
  -DORBBEC_SDK_ROOT="$ORBBEC_SDK"
cmake --build build-ego-py3.11 \
  --target ego_camera_plugin ego_mcap_export_media ego_mcap_merge --parallel
```

After installation, `ego.sh` is installed beside the Ego plugin and
supports `capabilities`, `record`, and `verify`. Its `build` command is only
for a source checkout.

The plugin uses the ordinary source build and install layout. The SDK remains
an explicit external dependency.

For SDK-free media tools and synthetic MCAP tests, configure separately:

```bash
cmake -S . -B build-ego-media-py3.11 \
  -DISAAC_TELEOP_PYTHON_VERSION=3.11 -DBUILD_VIZ=OFF \
  -DBUILD_PLUGIN_EGO_CAMERA=OFF -DBUILD_EGO_MEDIA_TOOLS=ON
cmake --build build-ego-media-py3.11 \
  --target ego_mcap_export_media ego_mcap_merge ego_mcap_tests --parallel
```

### Hardware-free software CI

The [software CI workflow](../../../.github/workflows/test-ego-software.yaml)
builds with an external SDK and runs without an attached camera. After building
with the SDK, run the same CTests locally:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  UV_CACHE_DIR=/tmp/isaaccapture-uv-cache \
  ctest --test-dir build-ego-py3.11 --output-on-failure --parallel \
  --no-tests=error -R '[Ee]go'
```

For viewer configuration tests, run
`uv run --python 3.11 --extra dev pytest` from
`tests/python/examples/camera_viz`.

From the repository root, run the review checks:

```bash
# Stage newly created project files first; --all-files enumerates tracked files.
SKIP=check-copyright-year pre-commit run --all-files
git diff --check
```

Software tests do not validate physical capture or XR. On the intended SDK,
firmware, and profile, verify completed recordings and exercise clean stop,
disconnect recovery, preview, and multi-device sessions.

Do not commit `recordings/`, `build/`, SDK extracts, `Log/`, or IDE files.

## Troubleshooting

| Symptom | Check |
|---|---|
| `doctor` reports no camera or no USB access | Install the selected SDK's udev rules, reconnect the camera, then run `doctor` and `capabilities` again. |
| Requested profile fails | Run `capabilities`; request one exact advertised profile per sensor. A profile may still fail as a simultaneous combination; the plugin does not fall back. |
| Recorded profile does not match `requested_profile.json` | Preserve the run and log. The requested profile was not delivered exactly; do not accept it as a silent substitute. |
| Embedded `verify --plugin PATH` cannot find an exporter | Build or install `ego_mcap_export_media` beside that plugin or in the same CMake build tree; do not mix artifacts from different builds. |
| `verify` reports no MCAP reader | Run `python3 -m pip install --user mcap`. |
| Video does not open in a desktop player | Validate/remux it with the FFmpeg commands above; elementary streams are not MP4 files. |
| `.partial` MCAP remains | The recording did not close cleanly. Preserve its log for diagnosis and do not publish it as a completed capture. |
| Capture waits after a disconnect | Reconnect the same physical device before `--reconnect-timeout` expires. The log reports any UID change, each attempt, and the resumed capture epoch. |
| Reconnect times out | Preserve the log and `.partial` MCAP for diagnosis, reconnect the camera, and start a new run. |
| An advertised encoded profile above 30 FPS is rejected | H.264/H.265 above 30 FPS is disabled. Choose an advertised profile at or below 30 FPS. |
| Preview is delayed | It is a CPU convenience preview. Close other GPU/camera applications or use `camera_viz` for the GPU path. |
| SchemaPusher exits immediately | A running OpenXR runtime is required; an old `cloudxr.env` file alone is insufficient. |
| SchemaPusher latency spikes at a regular multi-second interval | Confirm the plugin includes the nonblocking periodic device-state path; synchronous SDK identity polling in the capture drain loop can stall all streams. |
