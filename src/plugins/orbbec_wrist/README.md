<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Orbbec Wrist Camera Plugin

Records one Orbbec Wrist Color stream and optional accelerometer/gyroscope data
in Isaac Capture. Supports local MCAP recording, live TeleopSession publication,
same-device recovery, and optional preview. Run shell examples from the repository root.

| Stream | Supported configuration |
|---|---|
| Color | Fixed 1600×1200; H.264 (default), H.265, or MJPEG; default 30 FPS. |
| Accelerometer | Enabled by default; 400 (default) or 1000 Hz; default 24 g range. |
| Gyroscope | Enabled with accelerometer; same requested rate; fixed 2000 dps range. |

Profiles must be advertised by the connected device; unsupported requests fail
without fallback. Audio, depth, IR, and raw image output are not provided.
Each camera connects independently through its own plugin instance.

## Install and Build

### One-command Installation

On Linux x86_64, install CMake ≥3.24 and `uv`, then run:

```bash
./src/plugins/orbbec_wrist/install.sh --jobs 8
```

The [installer](install.sh) installs native dependencies, downloads and verifies
the pinned SDK, installs USB udev rules, and builds the executable. Run as your
normal user in an interactive terminal; system packages and host udev setup need
sudo access. Reconnect the camera afterward. Initial configuration also fetches
repository dependencies and requires network access.

The installer selects SDK 2.10.6 from the
[pinned vendor release](https://github.com/orbbec/OrbbecSDK_v2-EGO/releases/tag/v2.10.6-EGO-20261019):

```text
Archive: OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64.tar.gz
SHA-256: 378761b4331b7c6ce75d3c7dc61817ed27a5cacb3265a77477587c4d9e3bf11a
```

To use an extracted package instead:

```bash
export ORBBEC_SDK_ROOT=/absolute/path/to/OrbbecSDK
./src/plugins/orbbec_wrist/install.sh --sdk-root "$ORBBEC_SDK_ROOT" --jobs 8
```

Use `--skip-deps` and `--skip-udev` only on prepared hosts. Containers need USB
access and permissions configured on the host; `--container` skips host udev
setup when automatic detection is unavailable. See `install.sh --help` for options.

### Prerequisites and SDK Setup

For manual native dependency setup on Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential ca-certificates cmake ccache clang-format-14 curl git \
  patchelf pkg-config libudev-dev libusb-1.0-0-dev nlohmann-json3-dev \
  libssl-dev ffmpeg jq usbutils glslang-tools libegl-dev libvulkan-dev \
  libwayland-dev libx11-dev libxcursor-dev libxext-dev libxi-dev \
  libxinerama-dev libxkbcommon-dev libxrandr-dev wayland-protocols
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
sudo install -m 0644 "$ORBBEC_SDK_ROOT/shared/99-obsensor-libusb.rules" /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=2bc5 --attr-match=idProduct=1203
```

Reconnect the camera. The rules must include Wrist VID/PID `2bc5:1203`.

Recording verification needs FFmpeg, FFprobe, Python 3, and Python `mcap`.
The installer does not install `mcap`; prepare and keep an environment active:

```bash
uv venv --python 3.11 .venv-orbbec-wrist
source .venv-orbbec-wrist/bin/activate
uv pip install --python "$(command -v python3)" 'mcap>=1.2,<2'
```

TOML loading requires Python 3.11+. For Python sessions, build Isaac Capture and
its generated tracker bindings using the
[source-build guide](../../../docs/source/development/build_from_source/index.rst).
The package's `orbbec-wrist` extra declares `mcap`. Live sessions also need the
[Isaac Capture OpenXR runtime](../../../docs/source/teleop/teleop_session.rst);
the OpenXR loader alone does not provide it.

### Rebuild and Select the Executable

The helper builds the native executable; Python bindings are built separately:

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh build \
  --sdk-root "$ORBBEC_SDK_ROOT" --preset py3.11 --jobs 8
```

`py3.12` and `py3.13` select separate build trees. For an existing CMake Tools
configuration, enable `BUILD_PLUGINS` and `BUILD_PLUGIN_ORBBEC_WRIST_CAMERA`,
set `ORBBEC_SDK_ROOT` and `ISAAC_TELEOP_PYTHON_VERSION`, then build
`orbbec_wrist_camera_plugin`. Preview additionally needs `ORBBEC_WRIST_ENABLE_PREVIEW=ON`.

`--install-prefix PATH` installs the `orbbec_wrist` component under
`PATH/plugins/orbbec_wrist`. Installed `list` and `record` commands use the
adjacent executable; installation and rebuilds require a source checkout.

For native commands below, select the executable and the SDK used at build time.
After the default SDK download:

```bash
export ORBBEC_SDK_ROOT="$PWD/build/wrist-sdk/v2.10.6-EGO-20261019/OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64"
export ORBBEC_WRIST_PLUGIN="$PWD/build-wrist-py3.11/src/plugins/orbbec_wrist/orbbec_wrist_camera_plugin"
export LD_LIBRARY_PATH="$ORBBEC_SDK_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

Adjust paths for a local SDK or installed executable. The helper's `--sdk-root`
sets its SDK loader path automatically.

## Usage

### Discover and Record

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh doctor --sdk-root "$ORBBEC_SDK_ROOT"
./src/plugins/orbbec_wrist/orbbec_wrist.sh list --sdk-root "$ORBBEC_SDK_ROOT"
"$ORBBEC_WRIST_PLUGIN" --list-capabilities
RUN="recordings/orbbec_wrist_$(date +%Y%m%d_%H%M%S)"
./src/plugins/orbbec_wrist/orbbec_wrist.sh record --sdk-root "$ORBBEC_SDK_ROOT" \
  --duration 10 --output "$RUN"
```

`doctor` checks setup and loading; `list` reports compatible cameras.
`--list-capabilities` opens the selected device and prints SDK/firmware and
advertised profiles. One compatible camera is selected automatically; use helper
`--serial-number SN` or native `--serial-number=SN` when several are connected.
Recovery stays pinned to the selected physical camera.

The helper records Color, IMU, calibration, and diagnostics, then verifies them.
The output directory must be new. Without `--duration`, capture waits for Ctrl-C;
press it once and allow cleanup and verification to finish.

For native video-only output:

```bash
"$ORBBEC_WRIST_PLUGIN" --format=h264 --fps=30 --disable-imu --duration=10 \
  --video-output="recordings/wrist_video_$(date +%Y%m%d_%H%M%S)/Color.h264"
```

Use `h265` with `.h265`, or `mjpg` with `.mjpg`. Resolution is fixed;
`--width` and `--height` (including TOML keys) are rejected. Use MCAP or a
session to retain IMU; video files alone cannot store its structured samples.

### Preview

Enable native SDL preview at build and recording time:

```bash
./src/plugins/orbbec_wrist/install.sh --sdk-root "$ORBBEC_SDK_ROOT" --preview --jobs 8
./src/plugins/orbbec_wrist/orbbec_wrist.sh record --sdk-root "$ORBBEC_SDK_ROOT" \
  --duration 10 --output "recordings/wrist_preview_$(date +%Y%m%d_%H%M%S)" --preview
```

Manual preview dependencies are `libsdl2-dev`, `libavcodec-dev`, `libavutil-dev`,
and `libswscale-dev`. A graphical desktop is required; native recording needs
neither a desktop nor CUDA. Esc/Q or window close stops capture and finalizes
output. Preview errors disable the window while recording continues.

For the independent GPU viewer, follow the
[camera_viz setup guide](../../../examples/camera_viz/README.md#setup-one-time):

```bash
examples/camera_viz/camera_viz.sh setup --with-orbbec-wrist --sdk-root "$ORBBEC_SDK_ROOT"
examples/camera_viz/camera_viz.sh run \
  examples/camera_viz/configs/orbbec_wrist.yaml --mode window
```

Set `serial_number` in the [viewer configuration](../../../examples/camera_viz/configs/orbbec_wrist.yaml)
when needed. H.264/H.265 preview requires CUDA/NVDEC. Run the viewer and recording
plugin one at a time; each opens the camera. Close the viewer or use Ctrl-C to exit.

### Record Data in MCAP

| Workflow | Native options | Result |
|---|---|---|
| Video sidecar only | `--video-output=PATH` | Accepted encoded Color bytes in a file. |
| Local MCAP | `--mcap-filename=PATH` | Video metadata and enabled IMU; sidecar video by default. Add `--media-mode=embedded` to store media in MCAP. |
| TeleopSession | `--collection-prefix=PREFIX` | Live metadata/IMU through OpenXR, available to generated trackers and session recording. |
| Local MCAP and TeleopSession | Both destination options | Local encoded video plus metadata/IMU publication. |

Local MCAP needs no OpenXR service unless publication or device status is requested.
A plain TeleopSession records metadata/IMU only. Use `embedded_recording` below
to include encoded video in the final session MCAP.

#### Local MCAP

Record Color video, enabled IMU, and calibration without an OpenXR service:

```bash
RUN="recordings/orbbec_wrist_mcap_$(date +%Y%m%d_%H%M%S)"
"$ORBBEC_WRIST_PLUGIN" --format=h264 --fps=30 --imu-rate=400 --duration=10 \
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
from pathlib import Path
import isaaccapture.deviceio_trackers as dt
from isaaccapture.deviceio_session import McapRecordingConfig
from isaaccapture.teleop_session_manager import PluginConfig
from isaaccapture.orbbec_wrist import embedded_recording

run = Path("recordings/teleop_orbbec_wrist").resolve()
run.mkdir(parents=True, exist_ok=True)
orbbec_wrist_entries = [
    (dt.OrbbecWristFrameMetadataTracker("orbbec_wrist/camera_1/Color"), "orbbec_wrist/camera_1"),
    (dt.OrbbecWristImuTracker("orbbec_wrist/camera_1/Accel"), "orbbec_wrist/camera_1/Accel"),
    (dt.OrbbecWristImuTracker("orbbec_wrist/camera_1/Gyro"), "orbbec_wrist/camera_1/Gyro"),
]
existing = config.mcap_config.get_tracker_names() if config.mcap_config else []
config.trackers.extend(tracker for tracker, _ in orbbec_wrist_entries)
config.mcap_config = McapRecordingConfig(str(run / "final.mcap"), existing + orbbec_wrist_entries)
config.plugins.append(PluginConfig(
    plugin_name="orbbec_wrist_camera",
    plugin_root_id="orbbec_wrist_1",
    search_paths=[Path("build-wrist-py3.11/src/plugins").resolve()],
    required=True,
    shutdown_timeout_seconds=5.0,
    plugin_args=[
        "--serial-number=YOUR_WRIST_SERIAL", "--collection-prefix=orbbec_wrist/camera_1",
        "--format=h264",
    ],
))
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
`.partial` paths must be unused. Inputs are retained. The plugin search path is
the parent of plugin folders (`INSTALL_PREFIX/plugins` after installation).
Catch `KeyboardInterrupt` inside the context as shown.

The example sets `shutdown_timeout_seconds=5.0` for this Wrist instance, allowing
camera and recording cleanup; the framework default remains 2 seconds. This
requires the configurable shutdown timeout API. Forced termination can leave
an uncommitted `.partial` fragment and prevent finalization.

For two cameras, add another plugin/tracker group with a distinct explicit
serial, root ID, collection prefix, recording names, and fragment path. Map both
root IDs in `media_fragments`. Instances do not synchronize their clocks.
The helper marks selected Wrist plugins as required and retains other session devices.

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

#### Availability Monitoring

After `session.step()`, query
`session.get_device_status("orbbec_wrist_1/camera/wrist")`. Starting/recovery
maps to `DEGRADED`, streaming to `CONNECTED`, faults to `FAILED`, and closure to
`DISCONNECTED`. Delivery depends on runtime polling; process health remains
available if device-status publication is unavailable. The status path is
`<plugin_root_id>/camera/wrist`; it identifies one plugin instance.

### Store Media Inside MCAP

Sidecar mode is the default: encoded `raw/Color.*` sits beside the local MCAP.
For embedded media, use helper `record --media-mode embedded` or native
`--media-mode=embedded`. Add helper `--keep-media-sidecars` or native
`--video-output=PATH` to retain a sidecar too. Verification compares both copies
when present. `--keep-media-sidecars` is a helper option only.

For one MCAP with video and other session devices, use the
[TeleopSession example](#teleopsession-one-mcap-with-other-trackers).
Shell `record --mcap-media-spool PATH` adds a fragment destination but still
writes/verifies its normal delivery; it does not start or merge a TeleopSession.

## Configuration

### Native CLI Options

The native executable uses `--option=value`. Helper options use `--option value`;
TOML is accepted only by `orbbec_wrist.sh record`.

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

### Helper Configuration

Pass `record --config PATH` using [orbbec_wrist.example.toml](orbbec_wrist.example.toml)
as a template. Its `[record]` table requests 30 seconds of H.264 with IMU enabled.
CLI values override TOML; relative paths use the invoking working directory.
Unknown keys, extra tables, and invalid types are rejected.

Use helper `--no-imu` for video-only capture. `--dry-run` prints the native command
without creating output or opening hardware, but requires an executable plugin.

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

The helper creates a complete delivery directory:

```text
recordings/orbbec_wrist_<timestamp>/
├── raw/Color.h264             # .h265 or .mjpg for the other formats
├── calibration.json
├── metadata.mcap
├── capabilities.txt
├── requested_profile.json
├── verification_report.txt    # Only after successful verification.
└── logs/capture.log
```

Native invocations omit the helper's requested-profile manifest, log, and report.

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

### Calibration and Verification

`calibration.json` and local MCAP metadata preserve available SDK intrinsics,
raw alignment/IMU YAML, and availability errors. Structured Color intrinsics
may be missing even when YAML is readable. Export alone does not prove complete
or accurate calibration. Require all supported calibration fields explicitly:

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh verify "$RUN" --require-calibration
```

The helper also accepts `record --require-calibration`. Without that flag,
`verify "$RUN"` rechecks an existing delivery using normal requirements.
Verification checks MCAP CRCs, requested profiles, device time/sequence continuity,
IMU streams, media correlation/sizes, and complete video decoding/counts.
Reports include SHA-256 fingerprints and observed sequence gaps. Success creates
`verification_report.txt`; failure creates `verification_report.failed.txt`
and removes stale success reports. Keep failed captures and logs.

For manual H.264 sidecar inspection and playback:

```bash
ffmpeg -v error -xerror -f h264 -i "$RUN/raw/Color.h264" -f null -
ffplay -f h264 -framerate 30 "$RUN/raw/Color.h264"
ffmpeg -f h264 -framerate 30 -i "$RUN/raw/Color.h264" -c:v copy "$RUN/Color.mp4"
```

Use a new MP4 path. For H.265 use `-f hevc` and `.h265`; for MJPEG use
`-f mjpeg` and transcode with `-c:v libx264 -pix_fmt yuv420p`. Elementary streams
are not MP4 files; the playback frame rate does not reconstruct device timing.
There is no `export-media` helper; retain sidecars or read embedded media below.

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

### Timestamps and Synchronization Limits

`device_timestamp_us` is SDK device time; `system_timestamp_us` is the SDK host
receipt time with an unknown OS clock origin. `retrieved_monotonic_ns` samples
Linux `CLOCK_MONOTONIC` at frameset retrieval. Record wrappers retain sample and
availability times separately; discontinuity flags do not establish clock mapping.

Device/global/external synchronization is not enabled. Exposure timing, IMU
axes, drift, cross-stream alignment, and SDK buffering/drop behavior are unknown.
IMU sequence gaps and device-clock jumps can occur; flags and verification do not
detect every anomaly. Inspect continuity separately. Neither a verification pass
nor two camera instances establishes zero loss or synchronized sampling.

## Troubleshooting

| Symptom | What to check |
|---|---|
| No device or permission denied | Install the selected SDK's Orbbec Wrist `2bc5:1203` udev rule, reconnect, and run `--list-devices`. Containers also need host USB access. |
| SDK checksum or file validation fails | Retry the installer or select a compatible local SDK; do not bypass integrity checks. |
| SDK library cannot load | Select the same external SDK used for the build and expose its `lib` directory through `--sdk-root` or the native loader path. |
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
