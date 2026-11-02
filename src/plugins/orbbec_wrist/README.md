<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Orbbec Wrist Camera Plugin

Records one Orbbec Wrist Color stream and optional accelerometer/gyroscope data
in Isaac Capture. Supports local MCAP recording, live TeleopSession publication,
same-device recovery, and optional preview. Run shell examples from the repository root.

Camera examples below invoke the native executable through `ORBBEC_WRIST_PLUGIN`
and use `--option=value`. The optional
[orbbec_wrist.sh quick start](#quick-start-with-orbbec_wristsh-optional)
automates recording setup and verification.

## Features

- Record MJPEG, H.264, or H.265 Color video as an elementary stream.
- Capture accelerometer and gyroscope data.
- Write structured data to a local MCAP or publish it to a multi-device TeleopSession.
- Embed encoded video in MCAP and merge it with session recordings without transcoding.
- Inspect device profiles and preview Color video.
- Recover the same physical camera after a disconnect.

The plugin does not provide audio, depth, IR, or raw image output.
Profiles must be advertised by the connected device; unsupported requests fail
without fallback. Each camera connects independently through its own plugin instance.

## Install and Build

### One-command Installation

On Linux x86_64, install CMake 3.24 or newer and `uv`, then run from the
Isaac Capture repository root:

```bash
./src/plugins/orbbec_wrist/install.sh
```

The [installer](install.sh) installs native dependencies, downloads and verifies
the pinned SDK, installs USB udev rules, and builds the executable. Run as your
normal user in an interactive terminal; system packages and host udev setup need
sudo access. Reconnect the camera afterward. Initial configuration also fetches
repository dependencies and requires network access.

After reconnecting, select the SDK and executable and inspect the camera:

```bash
export ORBBEC_SDK_ROOT="$PWD/build/wrist-sdk/v2.10.6-EGO-20261019/OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64"
export ORBBEC_WRIST_PLUGIN="$PWD/build-wrist-py3.11/src/plugins/orbbec_wrist/orbbec_wrist_camera_plugin"
"$ORBBEC_WRIST_PLUGIN" --list-capabilities
```

Adjust paths for a local SDK or installed executable. Installed plugins live
under `INSTALL_PREFIX/plugins/orbbec_wrist`.

To use an extracted package instead:

```bash
export ORBBEC_SDK_ROOT=/absolute/path/to/OrbbecSDK
./src/plugins/orbbec_wrist/install.sh --sdk-root="$ORBBEC_SDK_ROOT"
```

Use `--skip-deps` and `--skip-udev` only on prepared hosts. Containers need USB
access and permissions configured on the host; `--container` skips host udev
setup when automatic detection is unavailable. See `install.sh --help` for options.

### Manual Prerequisites

The installer selects SDK 2.10.6 from the
[pinned vendor release](https://github.com/orbbec/OrbbecSDK_v2-EGO/releases/tag/v2.10.6-EGO-20261019):

```text
Archive: OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64.tar.gz
SHA-256: 378761b4331b7c6ce75d3c7dc61817ed27a5cacb3265a77477587c4d9e3bf11a
```

For a manual setup, download that archive from the release page, then extract it
and select the SDK root:

```bash
tar -xzf OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64.tar.gz
export ORBBEC_SDK_ROOT="$PWD/OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64"
```

For manual native dependency setup on Ubuntu:

```bash
sudo apt-get update
sudo apt install -y build-essential cmake ccache clang-format-14 patchelf \
  pkg-config libudev-dev nlohmann-json3-dev libssl-dev ffmpeg jq usbutils
```

Install `uv` using its [installation guide](https://docs.astral.sh/uv/getting-started/installation/)
and run `uv python install 3.11`. Ubuntu 22.04's CMake 3.22 is too old;
`uv tool install 'cmake>=3.24,<4'` provides a newer version on the user tool path.
Keep repository-managed dependencies at their pinned versions.

The external SDK root must contain:

```text
include/libobsensor/ObSensor.hpp
lib/OrbbecSDKConfig.cmake
lib/libOrbbecSDK.so.2
lib/extensions/
shared/99-obsensor-libusb.rules
```

All Orbbec plugins in one build tree must share the same `ORBBEC_SDK_ROOT` with
the APIs each requires. Keep SDK libraries and extensions together. SDK files
are not bundled with the plugin or wheel; redistribution permission is unknown.
For manual host USB setup, use the rules from that package:

```bash
sudo "$ORBBEC_SDK_ROOT/shared/install_udev_rules.sh"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reconnect the camera. The rules must include Wrist VID/PID `2bc5:1203`.

For Python sessions, build Isaac Capture and its generated tracker bindings using
the [source-build guide](../../../docs/source/development/build_from_source/index.rst).
The package's `orbbec-wrist` extra declares `mcap`. Live sessions also need the
[Isaac Capture OpenXR runtime](../../../docs/source/teleop/teleop_session.rst);
the OpenXR loader alone does not provide it. Native recording does not require Python.

For SDL preview, also install:

```bash
sudo apt install -y libsdl2-dev libavcodec-dev libavutil-dev libswscale-dev
```

### Manual Configure and Build

Run from the Isaac Capture repository root, keeping `ORBBEC_SDK_ROOT` set to
the extracted SDK path above:

```bash
cmake -S . -B build-wrist-py3.11 \
  -DISAAC_TELEOP_PYTHON_VERSION=3.11 \
  -DBUILD_VIZ=OFF -DBUILD_PLUGINS=ON -DBUILD_PLUGIN_ORBBEC_WRIST_CAMERA=ON \
  -DORBBEC_SDK_ROOT="$ORBBEC_SDK_ROOT"
cmake --build build-wrist-py3.11 \
  --target orbbec_wrist_camera_plugin --parallel

export ORBBEC_WRIST_PLUGIN="$PWD/build-wrist-py3.11/src/plugins/orbbec_wrist/orbbec_wrist_camera_plugin"
"$ORBBEC_WRIST_PLUGIN" --help
```

Add `-DORBBEC_WRIST_ENABLE_PREVIEW=ON` to the configure command for SDL preview.
Recording alone does not require SDL2 or FFmpeg development packages.
The build executable locates the SDK through its embedded RUNPATH; no
`LD_LIBRARY_PATH` setting is needed while the SDK remains at its build-time path.
The native executable installs under `plugins/orbbec_wrist`; after installation,
set `ORBBEC_WRIST_PLUGIN` to that executable. Installed binaries have no SDK
RUNPATH; see troubleshooting if the external SDK cannot be found.
Python bindings are built separately using the source-build guide above.

## Usage

### Inspect the Camera

```bash
"$ORBBEC_WRIST_PLUGIN" --list-devices
"$ORBBEC_WRIST_PLUGIN" --list-capabilities
```

`--list-devices` lists compatible cameras. `--list-capabilities` opens the selected
device and prints SDK/firmware and advertised profiles. One compatible camera is
selected automatically; add `--serial-number=YOUR_WRIST_SERIAL` to capability and
recording commands when several are connected. Recovery stays pinned to that camera.

### Record Color Video

```bash
RUN="recordings/orbbec_wrist_video_$(date +%Y%m%d_%H%M%S)"
"$ORBBEC_WRIST_PLUGIN" --format=h264 --fps=30 --disable-imu --duration=30 \
  --video-output="$RUN/raw/Color.h264"
```

Use fresh output paths. Without `--duration`, capture waits for Ctrl-C; press it
once and allow cleanup to finish. Native recording does not automatically run
the optional helper's delivery verification.

This command records video only. Native defaults are H.264, fixed 1600×1200 at
30 FPS, IMU enabled, and sidecar media. Use MCAP or TeleopSession to retain IMU;
video files alone cannot store its structured samples.

### Select a Profile or Device

For several connected cameras, select a device explicitly:

```bash
RUN="recordings/orbbec_wrist_selected_$(date +%Y%m%d_%H%M%S)"
"$ORBBEC_WRIST_PLUGIN" --serial-number=YOUR_WRIST_SERIAL \
  --format=h265 --fps=30 --disable-imu --duration=30 \
  --video-output="$RUN/raw/Color.h265"
```

Use `h265` with `.h265`, or `mjpg` with `.mjpg`. Resolution is fixed;
`--width` and `--height` (including TOML keys) are rejected.

### Optional IMU and Preview

IMU is enabled by default. `--disable-imu` disables both streams;
`--imu-rate=400|1000` selects their requested rate, and `--accel-full-scale=16|24`
selects the accelerometer range. Gyro range is fixed at 2000 dps. Add
`--calibration-output="$RUN/calibration.json"` to export available calibration.

Build with `ORBBEC_WRIST_ENABLE_PREVIEW=ON`, or add `--preview` to the installer,
then request a live Color window:

```bash
"$ORBBEC_WRIST_PLUGIN" --format=h264 --fps=30 --duration=30 --preview \
  --mcap-filename="recordings/wrist_preview_$(date +%Y%m%d_%H%M%S)/metadata.mcap"
```

Preview dependencies are listed under manual prerequisites. A graphical desktop
is required; native recording needs neither a desktop nor CUDA. Esc/Q or window close stops capture and finalizes
output. Preview errors disable the window while recording continues.

For the independent GPU viewer, follow the
[camera_viz setup guide](../../../examples/camera_viz/README.md#setup-one-time)
and enable its Orbbec Wrist backend.

Set `serial_number` in the [viewer configuration](../../../examples/camera_viz/configs/orbbec_wrist.yaml)
when needed. H.264/H.265 preview requires CUDA/NVDEC. Run the viewer and recording
plugin one at a time; each opens the camera. Close the viewer or use Ctrl-C to exit.

### Audio and Synchronization Limits

Audio capture and output are not supported. Device/global/external synchronization
is not enabled; separate camera instances do not synchronize their clocks.
Verification does not establish zero loss or synchronized sampling. Timestamp
details appear under structured replay and offline reading.

### Record Data in MCAP

| Workflow | Native options | Result |
|---|---|---|
| Video sidecar only | `--video-output=PATH` | Accepted encoded Color bytes in a file. |
| Local MCAP | `--mcap-filename=PATH` | Video metadata and enabled IMU; sidecar video by default. Add `--media-mode=embedded` to store media in MCAP. |
| TeleopSession | `--collection-prefix=PREFIX` | Live metadata/IMU through OpenXR, available to generated trackers and session recording. |
| Local MCAP and TeleopSession | Both destination options | Local encoded video plus metadata/IMU publication. |

Local MCAP needs no OpenXR service unless publication or device status is requested.
A plain TeleopSession records metadata/IMU only. Use `embedded_recording` in the
embedded-media section to include encoded video in the final session MCAP.

#### Local MCAP

Record Color video, enabled IMU, and calibration without an OpenXR service:

```bash
RUN="recordings/orbbec_wrist_mcap_$(date +%Y%m%d_%H%M%S)"
"$ORBBEC_WRIST_PLUGIN" --format=h264 --fps=30 --imu-rate=400 --duration=30 \
  --mcap-filename="$RUN/metadata.mcap" \
  --calibration-output="$RUN/calibration.json" --capabilities-output="$RUN/capabilities.txt"
```

Default sidecar mode creates `raw/Color.*` beside the MCAP. Use `--disable-imu`
to omit both IMU streams. Capture requires at least one of `--mcap-filename`,
`--collection-prefix`, or `--video-output`.

#### TeleopSession: one MCAP with other trackers

For an existing live `TeleopSessionConfig` named `config`, with its other devices
and pipeline already configured, append the following before opening the session.
The names `camera_1` and `orbbec_wrist_1` are user-defined identifiers for one
independent camera instance; they do not assign a physical side or pair cameras:

```python
import os
from pathlib import Path
import isaaccapture.deviceio_trackers as dt
from isaaccapture.deviceio_session import McapRecordingConfig
from isaaccapture.teleop_session_manager import PluginConfig, TeleopSession

plugin = Path(os.environ["ORBBEC_WRIST_PLUGIN"]).resolve()
run = Path("recordings/teleop_orbbec_wrist").resolve()
run.mkdir(parents=True, exist_ok=True)
orbbec_wrist_entries = [
    (dt.OrbbecWristFrameMetadataTracker("orbbec_wrist/camera_1/Color"), "orbbec_wrist/camera_1"),
    (dt.OrbbecWristImuTracker("orbbec_wrist/camera_1/Accel"), "orbbec_wrist/camera_1/Accel"),
    (dt.OrbbecWristImuTracker("orbbec_wrist/camera_1/Gyro"), "orbbec_wrist/camera_1/Gyro"),
]
existing = config.mcap_config.get_tracker_names() if config.mcap_config else []
config.trackers.extend(tracker for tracker, _ in orbbec_wrist_entries)
config.mcap_config = McapRecordingConfig(str(run / "metadata.mcap"), existing + orbbec_wrist_entries)
config.plugins.append(PluginConfig(
    plugin_name="orbbec_wrist_camera",
    plugin_root_id="orbbec_wrist_1",
    search_paths=[plugin.parent.parent],
    required=True,
    shutdown_timeout_seconds=5.0,
    plugin_args=[
        "--serial-number=YOUR_WRIST_SERIAL",
        "--collection-prefix=orbbec_wrist/camera_1",
        "--format=h264",
    ],
))
with TeleopSession(config) as session:
    try:
        while True:
            session.step()
    except KeyboardInterrupt:
        pass
```

This records structured metadata and enabled IMU with the other trackers; encoded
video is not included. To include video in the final MCAP, use the embedded workflow
below. Use absolute output paths. Plugin search paths point to the parent of plugin
folders (`INSTALL_PREFIX/plugins` after installation).

The example sets `shutdown_timeout_seconds=5.0` for this Wrist instance, allowing
camera and recording cleanup; the framework default remains 2 seconds. This
requires the configurable shutdown timeout API. Forced termination can leave
unfinished outputs; embedded capture can leave an uncommitted `.partial` fragment.

For two cameras, add another plugin/tracker group with a distinct explicit
serial, root ID, collection prefix, and recording names. Instances do not synchronize their clocks.

After `session.step()`, query
`session.get_device_status("orbbec_wrist_1/camera/wrist")`. Starting/recovery
maps to `DEGRADED`, streaming to `CONNECTED`, faults to `FAILED`, and closure to
`DISCONNECTED`. Delivery depends on runtime polling; process health remains
available if device-status publication is unavailable. The status path is
`<plugin_root_id>/camera/wrist`; it identifies one plugin instance.

### Store Media Inside MCAP

Media layout is separate from the recording workflow:

| Layout | Option | Media location |
|---|---|---|
| Sidecar (default) | `--media-mode=sidecar` | Encoded `raw/Color.*` beside local MCAP. |
| Embedded | `--media-mode=embedded` | Encoded Color video inside MCAP, without transcoding. |

For a self-contained local recording:

```bash
RUN="recordings/orbbec_wrist_embedded_$(date +%Y%m%d_%H%M%S)"
"$ORBBEC_WRIST_PLUGIN" --format=h264 --fps=30 --duration=30 \
  --media-mode=embedded --mcap-filename="$RUN/metadata.mcap" \
  --calibration-output="$RUN/calibration.json" --capabilities-output="$RUN/capabilities.txt"
```

Add `--video-output=PATH` to retain a sidecar too. For embedded multi-device
recording, replace the execution block of the TeleopSession example above with:

```python
from isaaccapture.orbbec_wrist import embedded_recording

config.mcap_config.filename = str(run / "final.mcap")
with embedded_recording(
    config, media_fragments={"orbbec_wrist_1": run / "camera_1.fragment.mcap"}
) as session:
    try:
        while True:
            session.step()
    except KeyboardInterrupt:
        pass
```

The helper copies the plugin configuration, adds embedded media/spool options,
records `final.session.mcap`, and merges committed camera fragments into
`final.mcap` after clean shutdown. All final, intermediate, fragment, and matching
`.partial` paths must be unused. Inputs are retained.
Catch `KeyboardInterrupt` inside the context as shown.

The helper marks selected Wrist plugins as required and retains other session
devices. For multiple cameras, map each root ID to a distinct fragment path in
`media_fragments`.

<details>
<summary>Advanced: fragment topics and manual finalization</summary>

Fragments contain Color metadata/video, IMU, calibration, identity, and terminal
diagnostics under camera-specific topics such as `orbbec_wrist/camera_1/Color`,
`orbbec_wrist/camera_1/Accel/Imu`, and `orbbec_wrist/camera_1/EncodedVideo/Color`.
Overlapping session observations with identical schemas move to `session_observed/`;
capture topics retain all samples, including those not polled by the session.
Incompatible schemas are rejected; payloads and timestamps are preserved without transcoding.

For manual finalization after clean shutdown, call
`isaaccapture.orbbec_wrist.merge_recording(session_path, fragment_paths, output_path)`.

Missing/partial fragments, identity mismatches, damaged MCAP, or missing clean
Closed events prevent finalization. Preserve inputs and partial output for diagnosis.

</details>

## Configuration

### Native CLI Options

Examples use `--option=value`; flags take no value. The native executable and
shell scripts also accept the space-separated form. TOML configuration is
available only through the [optional helper](#quick-start-with-orbbec_wristsh-optional).

| Option | Native default | Purpose |
|---|---|---|
| `--serial-number=SN` | Automatic with one compatible Orbbec Wrist | Required when several compatible devices are connected. |
| `--list-devices` | Off | List compatible Orbbec Wrist identities and exit. |
| `--list-capabilities` | Off | Open the selected Orbbec Wrist, print advertised profiles, then exit. |
| `--format=h264\|h265\|mjpg` | `h264` | Request an encoded Color format; native `mjpeg` is also accepted. |
| `--fps=N` | `30` | Video rate; resolution is fixed at 1600×1200. Zero selects an advertised rate. |
| `--disable-imu` | IMU enabled | Explicit video-only capture. |
| `--imu-rate=400\|1000` | `400` Hz | Request the same rate for accelerometer and gyroscope. |
| `--accel-full-scale=16\|24` | `24` g | Request the accelerometer range; gyro is fixed at 2000 dps. |
| `--media-mode=sidecar\|embedded` | `sidecar` | Choose raw Color media or embedded MCAP media. |
| `--calibration-output=PATH` | None | Save available structured intrinsics, raw YAML and availability errors. |
| `--capabilities-output=PATH` | None | Save SDK/firmware identity and advertised profiles. |
| `--mcap-media-spool=PATH` | None | Additional namespaced embedded MCAP fragment; requires `--media-mode=embedded` and `--collection-prefix`. The Python helper merges it after session shutdown. |
| `--mcap-filename=PATH` | None | Record metadata and enabled IMU; encoded bytes are embedded only in embedded mode. |
| `--preview` | Off | Show live Color in a preview-enabled build; Esc/Q or window close ends capture. |
| `--video-output=PATH` | None | Also write accepted encoded Color bytes to a sidecar, or use it as the sole output. |
| `--collection-prefix=PREFIX` | None | Publish Color metadata and enabled IMU through OpenXR. |
| `--plugin-root-id=ID` | None | Enable best-effort device-status publication for `/camera/wrist`. |
| `--duration=N` | `0` seconds | Stop after N seconds of the post-startup capture loop; zero waits for a signal. |
| `--startup-timeout-ms=N` | `5000` | Readiness deadline for requested streams. |
| `--stall-timeout-ms=N` | `3000` | Per-stream liveness deadline. |
| `--reconnect-timeout-ms=N` | `30000` | Same-device recovery deadline; zero disables recovery. |
| `--reconnect-interval-ms=N` | `1000` | Positive interval between recovery attempts. |

### Device Controls

The native CLI exposes the video format/FPS and IMU rate/range controls above.
It does not expose exposure, gain, white balance, bitrate, or generic SDK property
options. Do not copy EGO device-control arguments into Wrist commands. Inspect
`--list-capabilities` before selecting supported video and IMU profiles.

## Architecture

```text
Orbbec Wrist -> OrbbecSDK -> WristCamera -> FrameSink
                                          |-> raw Color / local MCAP
                                          `-> SchemaPusher -> trackers -> session MCAP
```

[core/](core/) contains acquisition and sinks. FrameSink removes timestamp SEI
and waits for H.264/H.265 parameter sets and keyframes after startup, gaps, or restarts.
Publication queue overflow stops capture. Recovery requires the original device
identity/profile and successful cleanup. Application deadlines cannot preempt SDK calls.

## Output and Verification

### Recording Files

The native local-MCAP example with calibration and capabilities output creates:

```text
recordings/orbbec_wrist_<timestamp>/
├── raw/Color.h264             # .h265 or .mjpg for the other formats
├── calibration.json
├── metadata.mcap
└── capabilities.txt
```

The [optional helper](#quick-start-with-orbbec_wristsh-optional) additionally saves
a requested-profile manifest, capture log, and verification report.

MCAP is created exclusively as `PATH.partial` and promoted after successful camera
shutdown, output closure, and commit. Existing final/partial files are not
overwritten. Promotion uses a hard link in the same directory; the filesystem
must support it. Failures can leave partial files; preserve them with the logs.

| Local MCAP topic | FlatBuffers record | Contents |
|---|---|---|
| `wrist_camera/Color` | `OrbbecWristFrameMetadataRecord` | Stream, sequence, profile, byte count, clocks, epoch, and discontinuity. |
| `orbbec_wrist_media/Color` | `OrbbecWristEncodedVideoFrameRecord` | Embedded mode only: encoded Color bytes and matching stream/sequence/epoch. |
| `wrist_camera/Accel/Imu` | `OrbbecWristImuBatchRecord` | Accelerometer batches when enabled. |
| `wrist_camera/Gyro/Imu` | `OrbbecWristImuBatchRecord` | Gyroscope batches when enabled. |
| `wrist_camera/Diagnostics` | JSON `wrist.CaptureStatus` | Capture state transitions and errors with host observation times. |

Schemas are defined in [orbbec_wrist_camera.fbs](../../core/schema/fbs/orbbec_wrist_camera.fbs)
and [orbbec_wrist_imu.fbs](../../core/schema/fbs/orbbec_wrist_imu.fbs); binary schemas
are embedded in MCAP. Correlate metadata and media by stream, sequence, and epoch.
IMU batches retain each sample's sequence, timestamps, temperature, and SI values.
They flush at 32 samples, 20 ms observed by a write, epoch changes, or close;
the time threshold is not a background timer.

### Calibration Data

`calibration.json` and local MCAP metadata preserve available SDK intrinsics,
raw alignment/IMU YAML, and availability errors. Structured Color intrinsics
may be missing even when YAML is readable. Export alone does not prove complete
or accurate calibration.

Use `--calibration-output="$RUN/calibration.json"` to export available calibration
as a sidecar. The optional helper creates this file automatically. Its
`--require-calibration` flag requires complete supported Color intrinsics and raw
calibration YAML; see the Quick Start section.

### Verify a Recording

For a completed H.264 sidecar recording, decode the video completely and inspect its profile:

```bash
ffmpeg -v error -xerror -f h264 -i "$RUN/raw/Color.h264" -f null -
ffprobe -v error -select_streams v:0 \
  -show_entries stream=codec_name,width,height -of default=noprint_wrappers=1 \
  "$RUN/raw/Color.h264"
```

A remaining `.partial` or nonzero capture exit means the recording did not finalize
successfully. Preserve data and logs for diagnosis. Full helper delivery verification
also checks MCAP, IMU, and media correlation; use the [optional helper](#quick-start-with-orbbec_wristsh-optional) workflow.
Native runs do not generate the `requested_profile.json` and `logs/capture.log`
required by that verifier. These checks do not establish physical cross-stream synchronization.

### Export and Play Media

Wrist has no `export-media` helper. To play or remux a retained H.264 sidecar:

```bash
ffplay -f h264 -framerate 30 "$RUN/raw/Color.h264"
ffmpeg -f h264 -framerate 30 -i "$RUN/raw/Color.h264" -c:v copy "$RUN/Color.mp4"
```

Use a new MP4 path. For H.265 use `-f hevc` and `.h265`; for MJPEG use
`-f mjpeg` and transcode with `-c:v libx264 -pix_fmt yuv420p`. Elementary streams
are not MP4 files; the playback frame rate does not reconstruct device timing.
For embedded recordings, read encoded media through the interface described below.

### Structured Replay and Offline Reading

The [tracker manifest](../../core/deviceio_trackers/trackers.toml) defines one
metadata tracker and one IMU tracker type. Create independent instances per
collection; there is no indexed multi-stream Orbbec Wrist consumer API.

| Live collection | Tensor | Generated consumer | Local replay topic prefix |
|---|---|---|---|
| `PREFIX/Color` | `frame_metadata` | `OrbbecWristFrameMetadataTracker("PREFIX/Color")` | `wrist_camera` |
| `PREFIX/Accel` | `imu_batch` | `OrbbecWristImuTracker("PREFIX/Accel")` | `wrist_camera/Accel` |
| `PREFIX/Gyro` | `imu_batch` | `OrbbecWristImuTracker("PREFIX/Gyro")` | `wrist_camera/Gyro` |

Configure these replay prefixes in `ReplayDeviceIOFactory` or the replay session's
tracker-name mapping. Collection IDs choose live tensors; recording names choose
MCAP prefixes. Generated replay reads raw `Color` or `Imu` channels one record per
instance per update, returns no payload at EOF, and does not reproduce session
update timing. Session recording also writes `Color_tracked` / `Imu_tracked`
snapshots; the replay manifest selects the raw channels.

The generated C++ includes are `<deviceio_trackers/orbbec_wrist_frame_metadata_tracker.hpp>`
and `<deviceio_trackers/orbbec_wrist_imu_tracker.hpp>`. C++ and Python expose
`get_data(session)` and retain the existing SchemaTracker/MCAP infrastructure.
Read encoded media separately with
`McapTrackerViewers<OrbbecWristEncodedVideoFrameRecord>`, topic prefix
`orbbec_wrist_media`, channel `Color`. Metadata replay does not decode video.

<details>
<summary>Advanced: timestamps and synchronization limits</summary>

`device_timestamp_us` is SDK device time; `system_timestamp_us` is the SDK host
receipt time with an unknown OS clock origin. `retrieved_monotonic_ns` samples
Linux `CLOCK_MONOTONIC` at frameset retrieval. Record wrappers retain sample and
availability times separately; discontinuity flags do not establish clock mapping.

Device/global/external synchronization is not enabled. Exposure timing, IMU
axes, drift, cross-stream alignment, and SDK buffering/drop behavior are unknown.
IMU sequence gaps and device-clock jumps can occur; flags and verification do not
detect every anomaly. Inspect continuity separately. Neither a verification pass
nor two camera instances establishes zero loss or synchronized sampling.

</details>

## Troubleshooting

| Symptom | What to check |
|---|---|
| No device or permission denied | Install the selected SDK's Orbbec Wrist `2bc5:1203` udev rule, reconnect, and run `--list-devices`. Containers also need host USB access. |
| SDK checksum or file validation fails | Retry the installer or select a compatible local SDK; do not bypass integrity checks. |
| SDK library cannot load | Check that the build-time SDK still exists. For installed binaries or a relocated SDK, add the selected SDK's `lib` directory to `LD_LIBRARY_PATH`, or use helper `--sdk-root`. |
| Requested profile rejected | Check the actual device's video/IMU profiles; defaults are requests. Use `--disable-imu` only for intentional video-only capture. |
| Camera is busy | Stop other processes opening that serial, including SDK viewers. Assign a distinct serial to each camera instance. |
| Recording waits after disconnect | Reconnect the same camera within `--reconnect-timeout-ms`; firmware/profile changes or cleanup failures prevent recovery. |
| Nonzero exit or `.partial` remains | Preserve capture and logs; check output errors, queue failures, timeout, and hard-link support. |
| Live trackers have no data | Check the OpenXR runtime, generated bindings, plugin search path, exact collection IDs, and enabled IMU. |
| Session MCAP has no encoded video | Use `embedded_recording` to merge committed media fragments, or retain a local plugin MCAP alongside publication; metadata trackers carry no video bytes. |
| Video does not open in a desktop player | Retain a sidecar, decode with FFmpeg, then remux/transcode with the correct format. |
| Unsupported option is rejected | Check the Orbbec Wrist native/help tables for supported options; the plugin provides one Color stream and no audio output. |
| Python MCAP reader is missing | Install `mcap` in the same `python3` environment used by the helper. Native installation does not install it. |
| Strict calibration fails | Inspect availability errors in `calibration.json`; missing SDK intrinsics require device/SDK investigation. |
| Verifier passes but IMU has gaps or clock jumps | Inspect sequence and timestamp continuity separately; delivery integrity is not a zero-loss or synchronization guarantee. |

## Quick Start with orbbec_wrist.sh (Optional)

After installation and exporting `ORBBEC_SDK_ROOT` and `ORBBEC_WRIST_PLUGIN` above,
the helper automates run-directory creation, recording, and verification. It
requests H.264 at 1600×1200 and 30 FPS, IMU at 400 Hz, calibration, capabilities,
and local MCAP with video sidecars by default.

Verification requires FFmpeg, FFprobe, and Python `mcap`; TOML loading requires
Python 3.11+. The installer does not install `mcap`. Prepare and keep an
environment active before using `record` or `verify`:

```bash
uv venv --python=3.11 .venv-orbbec-wrist
source .venv-orbbec-wrist/bin/activate
uv pip install --python="$(command -v python3)" 'mcap>=1.2,<2'
```

Then check the setup and record a new delivery:

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh doctor --plugin="$ORBBEC_WRIST_PLUGIN"
RUN="recordings/orbbec_wrist_helper_$(date +%Y%m%d_%H%M%S)"
./src/plugins/orbbec_wrist/orbbec_wrist.sh record --plugin="$ORBBEC_WRIST_PLUGIN" \
  --duration=30 --output="$RUN"
./src/plugins/orbbec_wrist/orbbec_wrist.sh record --help
```

The helper's `--sdk-root=PATH` overrides `ORBBEC_SDK_ROOT` and configures the SDK
loader path. `list` lists devices; `doctor` checks setup and loading. To rebuild
from a source checkout, use
`orbbec_wrist.sh build --sdk-root="$ORBBEC_SDK_ROOT" --preset=py3.11 --jobs=8`;
add `--preview` for SDL preview. `--install-prefix=PATH` on `build` or the
installer also installs the plugin component without SDK files. Installed
`list` and `record` commands default to the adjacent executable.

**Configuration and Recording Options**

For repeatable settings, copy [orbbec_wrist.example.toml](orbbec_wrist.example.toml)
outside the source tree. Its `[record]` table requests 30 seconds of H.264 with
IMU enabled. CLI values override TOML regardless of the position of `--config`;
relative paths use the invoking working directory. Unknown keys, extra tables,
and invalid types are rejected. Use a fresh output directory for each run:

```bash
RUN="recordings/orbbec_wrist_config_$(date +%Y%m%d_%H%M%S)"
./src/plugins/orbbec_wrist/orbbec_wrist.sh record --config=/path/to/run.toml \
  --plugin="$ORBBEC_WRIST_PLUGIN" --duration=60 --output="$RUN"
```

Use `--no-imu` for video-only capture and `--preview` for a preview-enabled
build. `--media-mode=embedded` stores video in MCAP; `--keep-media-sidecars`
retains a sidecar too. `--dry-run` prints the native command without creating
output or opening hardware, but requires an executable plugin. Omitting
`--duration` records until Ctrl-C; press it once and wait for cleanup and verification.
The duration measures the post-startup capture loop, as in the native executable.

`record --mcap-media-spool=PATH` adds a fragment destination but still writes and
verifies the normal delivery. It requires `--media-mode=embedded` and
`--collection-prefix=PREFIX`; it does not start or merge a TeleopSession.

**Delivery Verification**

Helper runs additionally save `requested_profile.json`, `logs/capture.log`, and
`verification_report.txt` after successful verification. To recheck a helper
delivery, including complete Color intrinsics and raw calibration YAML:

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh verify "$RUN" --require-calibration
```

Use `record --require-calibration` to require complete calibration immediately
after recording, or omit the flag for normal verification. Checks cover MCAP
CRCs, requested profiles, device time/sequence continuity, IMU streams, media
correlation/sizes, and complete video decoding/counts. Reports include SHA-256
fingerprints and observed sequence gaps. Failure creates
`verification_report.failed.txt` and removes stale success reports. Keep failed
captures and logs; a pass does not establish zero loss or synchronized sampling.
