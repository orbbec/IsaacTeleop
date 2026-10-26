<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Orbbec Wrist Camera Plugin

C++ plugin that records one Orbbec Wrist Color video stream and optional accelerometer
and gyroscope data in Isaac Capture through the Orbbec SDK.

The examples below use the native `orbbec_wrist_camera_plugin` executable. The optional
[orbbec_wrist.sh](orbbec_wrist.sh) helper provides build, setup checks, device discovery, and
recording commands. Run shell examples from the Isaac Capture repository root.

## Features

- Record fixed 1600×1200 MJPEG, H.264, or H.265 Color video; raw sidecars are the default.
- Select the only compatible Orbbec Wrist automatically; require a serial number when several are connected.
- Save calibration, capabilities, requested profiles, capture logs, and a verified delivery report.
- Capture independent accelerometer and gyroscope streams, enabled by default.
- Publish video metadata and IMU through SchemaPusher for generated trackers and session recording.
- Record local encoded media while publishing structured data to a TeleopSession.
- Recover the same physical camera after a disconnect, with a new capture epoch.
- Preview live Color in an optional SDL2 window while recording.
- Use two independent instances for a left/right wrist pair.

Current interfaces cover one Color stream and IMU at 400 or 1000 Hz. Audio, depth,
IR, and raw image output are not provided. Requested codecs, FPS and IMU ranges
are checked against the connected device without profile fallback. Calibration
export preserves available SDK intrinsics and raw YAML, with explicit errors for
unavailable fields; export alone does not prove that calibration is complete.

## Install and Build

### One-command Installation

On Linux x86_64, install CMake 3.24 or newer and `uv`, then run:

```bash
./src/plugins/orbbec_wrist/install.sh --jobs 8
```

The [installer](install.sh) installs missing native dependencies, downloads the
pinned Orbbec SDK, verifies its SHA-256, installs the vendor udev file, and invokes
`orbbec_wrist.sh build`. Run without sudo; only system package installation and
host udev setup request elevated access.
Reconnect the camera after installing the rules.

The default package is OrbbecSDK 2.10.6 from the
[pinned vendor release](https://github.com/orbbec/OrbbecSDK_v2-EGO/releases/tag/v2.10.6-EGO-20261019):

```text
Archive: OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64.tar.gz
SHA-256: 378761b4331b7c6ce75d3c7dc61817ed27a5cacb3265a77477587c4d9e3bf11a
```

Downloads are cached under `build/wrist-sdk/v2.10.6-EGO-20261019/`. Each run verifies
the archive, replaces a corrupt cache, and recreates the extracted SDK from the
verified archive. To use an existing package instead:

```bash
./src/plugins/orbbec_wrist/install.sh --sdk-root /absolute/path/to/OrbbecSDK --jobs 8
```

`ORBBEC_SDK_ROOT` also selects a local package. Use `--skip-deps` or `--skip-udev`
on a prepared host. Containers skip host udev setup; use `--container` if automatic
detection is unavailable. USB access and permissions must still be set up on the
host. Run `install.sh --help` for the supported build options.

### Manual Prerequisites

The steps below target Ubuntu on Linux x86_64 and match the
[installer's dependency list](install.sh). For the full Python package build,
also follow the [Isaac Capture source-build guide](../../../docs/source/development/build_from_source/index.rst).

#### 1. Install system libraries and build tools

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential ca-certificates cmake ccache clang-format-14 curl git \
  patchelf pkg-config libudev-dev libusb-1.0-0-dev nlohmann-json3-dev \
  libssl-dev ffmpeg jq usbutils glslang-tools libegl-dev libvulkan-dev \
  libwayland-dev libx11-dev libxcursor-dev libxext-dev libxi-dev \
  libxinerama-dev libxkbcommon-dev libxrandr-dev wayland-protocols
```

| Dependencies | Purpose |
|---|---|
| C++20 compiler, CMake ≥3.24, Git, pkg-config, ccache, clang-format-14, patchelf | Compile, configure, format, and package native targets. |
| `libudev-dev`, `libusb-1.0-0-dev`, `usbutils` | USB dependencies and device/access diagnostics. |
| `nlohmann-json3-dev`, `libssl-dev` | JSON headers and TLS development libraries used by the build. |
| EGL, Vulkan, X11, Wayland libraries, `wayland-protocols`, `glslang-tools` | Repository graphics/window build dependencies included by the installer. |
| `ffmpeg` | Installs FFmpeg and FFprobe for complete video decoding and delivery verification. |

These are the installer's prepared-host dependencies, rather than a minimal
runtime-only package set. Native recording does not require CUDA or a graphical
desktop. SDL preview and camera_viz have additional requirements below.

Install `uv`, add its executable directory to the current shell, and prepare
Python 3.11:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
uv python install 3.11
cmake --version
```

Ubuntu 22.04's default CMake 3.22 is too old. If the version printed above is
below 3.24, install a newer CMake on the user tool path:

```bash
uv tool install 'cmake>=3.24,<4'
hash -r
cmake --version
```

The repository fetches its pinned OpenXR SDK/loader, FlatBuffers, MCAP,
spdlog, yaml-cpp, and, when bindings are enabled, pybind11 during configuration.
Do not install arbitrary system versions to replace these managed dependencies.
The first configuration needs network access even after the Orbbec SDK has
been downloaded. Versions are defined in
[third-party CMake configuration](../../../deps/third_party/CMakeLists.txt).

#### 2. Install the external Orbbec SDK and USB rules

`install.sh` downloads and verifies the pinned SDK automatically. For manual
setup, download the Linux x86_64 archive linked under
[One-command Installation](#one-command-installation), check the SHA-256 shown
there, extract it, and set its absolute path:

```bash
mkdir -p /absolute/path/to/sdk-directory
tar -xzf /absolute/path/to/OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64.tar.gz \
  -C /absolute/path/to/sdk-directory
export ORBBEC_SDK_ROOT=/absolute/path/to/sdk-directory/OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64
```

Use an external SDK package containing:

```text
include/libobsensor/ObSensor.hpp
lib/OrbbecSDKConfig.cmake
lib/libOrbbecSDK.so.2
lib/extensions/
shared/99-obsensor-libusb.rules
```

SDK 2.9.0 was used for the comprehensive hardware test batch. The pinned
2.10.6 package was subsequently downloaded, built in a fresh directory, and used
for a ten-second hardware recording on WristCam-L firmware 1.1.8. This does not
validate every profile or long-duration operation. See
[Development Validation Status](#development-validation-status) for evidence.
Other packages and architectures require separate validation. All Orbbec camera
plugins enabled in one build tree must use the same `ORBBEC_SDK_ROOT`, containing
the APIs required by each enabled plugin.

The installer checks for an uncommented Orbbec Wrist USB VID/PID `2bc5:1203` rule in the
vendor udev file. That file covers other Orbbec devices too. For manual host setup,
using the same extracted package selected for the build:

```bash
sudo install -m 0644 "$ORBBEC_SDK_ROOT/shared/99-obsensor-libusb.rules" /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=2bc5 --attr-match=idProduct=1203
```

Reconnect Orbbec Wrist afterward. Keep the SDK's libraries and extensions together in
the external package. No SDK files are installed with the plugin or added to a
wheel. SDK binary/extension redistribution permission remains **UNKNOWN**.

#### 3. Install Python dependencies for recording verification

The recording helper's verifier requires FFmpeg, FFprobe, Python 3, and `mcap`.
The installer does not install the Python dependency. From the repository root,
create a dedicated environment, activate it, and install into the `python3`
interpreter used by the helper:

```bash
uv venv --python 3.11 .venv-orbbec-wrist
source .venv-orbbec-wrist/bin/activate
uv pip install --python "$(command -v python3)" 'mcap>=1.2,<2'
python3 -c 'from mcap.reader import make_reader; print("MCAP reader ready")'
ffmpeg -version
ffprobe -version
```

For an existing environment, activate it and run only the installation/check
commands. Keep that environment active when using `record` or `verify`.
`mcap` installs its Python compression dependencies through the package manager;
no separate system MCAP library is required. TOML configuration uses Python
3.11+'s standard-library `tomllib` and needs no extra TOML package.

For a session environment containing the built Isaac Capture package, the
`orbbec-wrist` extra declares this dependency. Live `TeleopSession` use additionally
requires the built Python tracker bindings and a running compatible OpenXR
runtime. Installing the fetched OpenXR loader alone does not supply that runtime;
follow the [session setup guide](../../../docs/source/teleop/teleop_session.rst).

#### 4. Install optional SDL preview libraries

```bash
sudo apt-get install -y libsdl2-dev libavcodec-dev libavutil-dev libswscale-dev
pkg-config --modversion sdl2 libavcodec libavutil libswscale
```

SDL2 creates the window; FFmpeg's codec, utility, and scaling libraries decode
and display encoded Color frames. Then enable preview when building with
`orbbec_wrist.sh build --preview`, and request it at runtime with
`record --preview`. `install.sh --preview` handles these packages and build
support together. Recording without preview does not require these development
packages; installing the `ffmpeg` command alone does not install their headers.

#### 5. Install optional camera_viz dependencies

The separate GPU viewer requires an NVIDIA GPU/driver, a compatible CUDA toolkit,
and the viewer's Python environment. H.264/H.265 decoding uses driver-provided
NVDEC (`libnvcuvid`). The shared GPU codec built by default also links NVENC
(`libnvidia-encode`), so both driver libraries must be available. Check the
driver and toolkit before setup:

```bash
nvidia-smi
nvcc --version
examples/camera_viz/camera_viz.sh setup --with-orbbec-wrist --sdk-root "$ORBBEC_SDK_ROOT"
```

The setup command creates its own environment, installs the viewer's declared
Python dependencies (including CuPy, NumPy, SciPy, PyYAML, and pybind11), and
builds the Orbbec Wrist binding. It also installs `isaaccapture[cloudxr]` for
the viewer; the native recording plugin does not need this extra. Missing system
packages are presented for confirmation. Follow the
[camera_viz setup guide](../../../examples/camera_viz/README.md#setup-one-time)
for driver/toolkit installation prerequisites and version compatibility. This
GPU viewer is independent of the CPU SDL preview above.

### Manual Configure and Build

With the repository's normal build dependencies, `uv`, and a supported Python
version available, the helper builds only the Orbbec Wrist executable:

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh build \
  --sdk-root "$ORBBEC_SDK_ROOT" --preset py3.11 --jobs 8
```

The default build tree is `build-wrist-py3.11`; `py3.12` and `py3.13` select their
corresponding Python versions and separate build trees. In CMake Tools, configure
`ISAAC_TELEOP_PYTHON_VERSION=3.11`, `BUILD_VIZ=OFF`, `BUILD_PLUGINS=ON`,
`BUILD_PLUGIN_ORBBEC_WRIST_CAMERA=ON`, and `ORBBEC_SDK_ROOT`, then list targets and build
`orbbec_wrist_camera_plugin`.

`--install-prefix PATH` on `install.sh` or `orbbec_wrist.sh build` installs component
`orbbec_wrist`: the executable, manifest, README, scripts, and example TOML under
`plugins/orbbec_wrist`. Installed `list` and `record` use the adjacent executable;
installation and builds require a source checkout. Build the Python package and
its generated bindings separately before using TeleopSession consumers.

For the native examples below, select the executable and the same SDK used at
build time. After the default automatic installation, the SDK path is:

```bash
export ORBBEC_SDK_ROOT="$PWD/build/wrist-sdk/v2.10.6-EGO-20261019/OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64"
export ORBBEC_WRIST_PLUGIN="$PWD/build-wrist-py3.11/src/plugins/orbbec_wrist/orbbec_wrist_camera_plugin"
export LD_LIBRARY_PATH="$ORBBEC_SDK_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$ORBBEC_WRIST_PLUGIN" --help
```

For a local SDK or an installed executable, change these paths accordingly.
The helper's `--sdk-root` prepends that package's `lib` to its loader path;
otherwise the loader must already resolve the external SDK.

Add `--preview` to the build helper for native SDL support. In CMake Tools,
set `ORBBEC_WRIST_ENABLE_PREVIEW=ON` (default: `OFF`).

## Usage

### Inspect the Camera

```bash
"$ORBBEC_WRIST_PLUGIN" --list-devices
"$ORBBEC_WRIST_PLUGIN" --list-capabilities
```

This lists compatible Orbbec Wrist devices (USB VID/PID `2bc5:1203`). Recording selects
one automatically when exactly one is present; with several devices, specify
`--serial-number SN`. An explicit serial must also identify a compatible Orbbec Wrist.
Selection is pinned before streaming, so recovery cannot switch to another camera.
The native `--list-capabilities` option opens the selected device, prints its SDK,
firmware and advertised profiles, then exits.

### Record Color Video

Create fresh output paths and record encoded Color video only:

```bash
RUN="recordings/orbbec_wrist_video_$(date +%Y%m%d_%H%M%S)"
"$ORBBEC_WRIST_PLUGIN" --format=h264 --fps=30 --disable-imu --duration=10 \
  --video-output="$RUN/raw/Color.h264"
```

Resolution is always 1600×1200. Native defaults are H.264 at 30 FPS, IMU enabled
at 400 Hz with 24 g / 2000 dps ranges, sidecar media, preview off, and unlimited
duration. The example explicitly disables IMU because a video file alone cannot
retain its structured samples. Use local MCAP or a session to record IMU.

Press Ctrl-C once to stop an unlimited capture. Wait for cleanup and the shell
prompt. Existing output files are not overwritten.

### Select a Profile or Device

Select a compatible serial and an advertised encoded profile:

```bash
"$ORBBEC_WRIST_PLUGIN" --serial-number=YOUR_WRIST_SERIAL \
  --format=h265 --fps=30 --disable-imu --duration=10 \
  --video-output="recordings/orbbec_wrist_h265_$(date +%Y%m%d_%H%M%S)/Color.h265"
```

Use `--format=mjpg` with a `.mjpg` file for MJPEG. `--width` and `--height`,
including TOML keys, are rejected. Requested codecs, FPS, IMU rates, and ranges
must be advertised by the device. A profile rejection does not select a fallback.

### Optional IMU and Preview

IMU is enabled by default; retain it through MCAP or a live session. Use
`--disable-imu` natively or `--no-imu` in the helper for intentional video-only
capture. Preview is enabled separately per recording.

Enable the window per recording with `--preview`, or `preview = true` in TOML:

```bash
RUN="recordings/orbbec_wrist_preview_$(date +%Y%m%d_%H%M%S)"
./src/plugins/orbbec_wrist/orbbec_wrist.sh record --serial-number YOUR_WRIST_SERIAL \
  --duration 10 --output "$RUN" --format h264 --preview
```

The native executable also accepts `--preview`. H.264, H.265 and MJPEG use the
same accepted packets as recording; the first image waits for a decodable frame.
The resizable window preserves aspect ratio. Press Esc/Q or close the window to
stop capture and finalize MCAP normally; Ctrl-C also remains available.

Preview needs a graphical desktop. Window, decoding, or queue failures log a
warning and disable preview while recording continues. Decoding runs on a worker
with a bounded queue; H.264/H.265 packets remain ordered, while stale MJPEG images
may be replaced. Rendering uses the latest decoded image, scaled to fit 800×600.
Preview does not establish camera/IMU synchronization or repair missing samples.

For the independent GPU camera_viz viewer:

```bash
examples/camera_viz/camera_viz.sh setup --with-orbbec-wrist --sdk-root "$ORBBEC_SDK_ROOT"
examples/camera_viz/camera_viz.sh run \
  examples/camera_viz/configs/orbbec_wrist.yaml --mode window
```

Set `serial_number` in the [viewer configuration](../../../examples/camera_viz/configs/orbbec_wrist.yaml)
when multiple cameras are connected. H.264/H.265 preview requires CUDA/NVDEC;
see the [camera_viz README](../../../examples/camera_viz/README.md) for setup.
The viewer and recording plugin each open the camera; run them one at a time.
Close the viewer window or use Ctrl-C to exit. Its Escape handling has an
unresolved test failure; this is separate from the SDL recording preview.

### Record Data in MCAP

| Workflow | Native options | Result |
|---|---|---|
| Video sidecar only | `--video-output=PATH` | Accepted encoded Color bytes in a file. |
| Local MCAP | `--mcap-filename=PATH` | Video metadata and enabled IMU; sidecar video by default. Add `--media-mode=embedded` to store media in MCAP. |
| TeleopSession | `--collection-prefix=PREFIX` | Live metadata/IMU through OpenXR, available to generated trackers and session recording. |
| Local MCAP and TeleopSession | Both destination options | Local encoded video plus metadata/IMU publication. |

Local MCAP needs no OpenXR service unless publication or device status is also
requested. Live publication requires the Isaac Capture OpenXR runtime and matching
tracker collection IDs; the Python session example also requires built Python
bindings. A plain `TeleopSession` records metadata and IMU; encoded video stays in the
raw sidecar or embedded local MCAP. For one final MCAP containing the video too,
use `isaaccapture.orbbec_wrist.embedded_recording` as shown below. Install the
`orbbec-wrist` package extra (`mcap>=1.2,<2`) for this Python workflow.

#### Local MCAP

Record Color video, enabled IMU, and calibration without an OpenXR service:

```bash
RUN="recordings/orbbec_wrist_mcap_$(date +%Y%m%d_%H%M%S)"
"$ORBBEC_WRIST_PLUGIN" --format=h264 --fps=30 --imu-rate=400 --duration=10 \
  --mcap-filename="$RUN/metadata.mcap" \
  --calibration-output="$RUN/calibration.json" --capabilities-output="$RUN/capabilities.txt"
```

With a local MCAP and default sidecar mode, omitting `--video-output` creates
`raw/Color.*` beside the MCAP. `--imu-rate=400` or `--imu-rate=1000` requests the
same rate for accelerometer and gyroscope; `--disable-imu` omits both.
At least one of `--mcap-filename`, `--collection-prefix`, or `--video-output`
is required for capture.

#### TeleopSession: one MCAP with other trackers

For an existing live `TeleopSessionConfig` named `config`, with its other devices
and pipeline already configured, append the following before opening the session:

```python
from pathlib import Path
import isaaccapture.deviceio_trackers as dt
from isaaccapture.deviceio_session import McapRecordingConfig
from isaaccapture.teleop_session_manager import PluginConfig
from isaaccapture.orbbec_wrist import embedded_recording

run = Path("recordings/teleop_orbbec_wrist").resolve()
run.mkdir(parents=True, exist_ok=True)
orbbec_wrist_entries = [
    (dt.OrbbecWristFrameMetadataTracker("orbbec_wrist/left/Color"), "orbbec_wrist/left"),
    (dt.OrbbecWristImuTracker("orbbec_wrist/left/Accel"), "orbbec_wrist/left/Accel"),
    (dt.OrbbecWristImuTracker("orbbec_wrist/left/Gyro"), "orbbec_wrist/left/Gyro"),
]
existing = config.mcap_config.get_tracker_names() if config.mcap_config else []
config.trackers.extend(tracker for tracker, _ in orbbec_wrist_entries)
config.mcap_config = McapRecordingConfig(str(run / "final.mcap"), existing + orbbec_wrist_entries)
config.plugins.append(PluginConfig(
    plugin_name="orbbec_wrist_camera",
    plugin_root_id="orbbec_wrist_left",
    search_paths=[Path("build-wrist-py3.11/src/plugins").resolve()],
    required=True,
    shutdown_timeout_seconds=15.0,
    plugin_args=[
        "--serial-number=YOUR_WRIST_SERIAL", "--collection-prefix=orbbec_wrist/left",
        "--format=h264",
    ],
))
with embedded_recording(
    config, media_fragments={"orbbec_wrist_left": run / "left.fragment.mcap"}
) as session:
    try:
        while True:
            session.step()
    except KeyboardInterrupt:
        pass
```

The helper adds `--media-mode=embedded` and `--mcap-media-spool=PATH` to a copy of
the plugin configuration. It records the session to `final.session.mcap`, waits
for clean plugin shutdown, then merges it with `left.fragment.mcap` into
`final.mcap`. All three paths and their `.partial` paths must be unused. Inputs
are retained after success and failure. The plugin search path
is the parent containing plugin folders; use `INSTALL_PREFIX/plugins` after
installation. Existing tracker recording names must remain unique. After stepping,
the tracker instances expose `get_data(session)`; a missing payload returns `None`
in Python. Frame metadata has no encoded bytes.

For two cameras, add another plugin and tracker group with a different serial,
`plugin_root_id` and `collection-prefix`, then supply both entries in
`media_fragments`, keyed by plugin root ID. The helper requires explicit distinct
serials for multiple cameras and marks every selected Orbbec Wrist plugin as required.
Other devices already present in the session configuration remain included.

Fragments contain complete accepted Color metadata, encoded Color video,
Accel/Gyro batches, calibration, device identity and terminal diagnostics. Topics
use each camera's collection prefix, for example `orbbec_wrist/left/Color`,
`orbbec_wrist/left/Accel/Imu` and `orbbec_wrist/left/EncodedVideo/Color`. Keep tracker recording
names aligned with these prefixes, as in the example. When session and capture
metadata/IMU topics overlap with identical schemas, the merger preserves session
observations under `session_observed/`; capture channels retain the original
topics and include samples not polled by the session. Incompatible topic schemas
are rejected. Payloads, log/publish timestamps, sequence numbers, metadata and
attachments are preserved; video is not decoded or transcoded.

For manual finalization of a two-camera capture after clean shutdown:

```python
from isaaccapture.orbbec_wrist import merge_recording

merge_recording(
    run / "final.session.mcap",
    [run / "left.fragment.mcap", run / "right.fragment.mcap"],
    run / "final.mcap",
)
```

A missing fragment, remaining `.partial`, mismatched device identity, damaged
MCAP or missing clean Closed event prevents publication of the final output.
Merge failures retain inputs and any partial output for diagnosis. The helper
requires a live `McapRecordingConfig`; catch `KeyboardInterrupt` inside its
context as shown. It finalizes Orbbec Wrist fragments only.

Keep the shutdown budget long enough for capture and output cleanup. The helper
has automated session/merge coverage, but real OpenXR and multiple physical
camera acceptance remain pending.

#### Availability Monitoring

Orbbec Wrist also participates in the session's device-status monitor. After `step()`,
`session.get_device_status("orbbec_wrist_left/camera/wrist")` reports that instance's
capture state; the session supplies its `plugin_root_id` to the plugin. Starting
and recovering map to `DEGRADED/NO_CURRENT_DATA`, streaming to `CONNECTED/NONE`,
faults to `FAILED/DEVICE_ERROR`, and closure to
`DISCONNECTED/NO_HARDWARE_SIGNAL`. Shutdown attempts to publish pending status
before destroying the transport; delivery to a consumer still depends on the
runtime and its polling. Process health remains available when status publication
is unavailable.

### Store Media Inside MCAP

Media layout is separate from the recording workflow:

| Layout | Native option | Media location |
|---|---|---|
| Sidecar (default) | `--media-mode=sidecar` | Encoded `raw/Color.*` beside the MCAP. |
| Embedded | `--media-mode=embedded` | Encoded Color records inside MCAP, without transcoding. |

For a self-contained local recording:

```bash
RUN="recordings/orbbec_wrist_embedded_$(date +%Y%m%d_%H%M%S)"
"$ORBBEC_WRIST_PLUGIN" --format=h264 --imu-rate=400 --duration=10 \
  --mcap-filename="$RUN/metadata.mcap" --media-mode=embedded \
  --calibration-output="$RUN/calibration.json"
```

Add native `--video-output="$RUN/raw/Color.h264"` to retain an encoded sidecar too.
With the helper, use `record --media-mode embedded`; add `--keep-media-sidecars`
to retain `raw/Color.*`. Verification compares embedded and retained media byte
for byte. `--keep-media-sidecars` is a helper option, not a native flag.

For one MCAP containing video and other session devices, use the
[TeleopSession example](#teleopsession-one-mcap-with-other-trackers). Its
`embedded_recording` helper supplies each plugin's media-spool options and
merges after clean shutdown; it supports multiple Orbbec Wrist instances.

### Use a Left/Right Orbbec Wrist Pair

Run one instance per physical camera, with different serial numbers and output
directories. For example, in separate terminals:

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh record --sdk-root "$ORBBEC_SDK_ROOT" \
  --serial-number LEFT_WRIST_SERIAL --output recordings/orbbec_wrist_left
./src/plugins/orbbec_wrist/orbbec_wrist.sh record --sdk-root "$ORBBEC_SDK_ROOT" \
  --serial-number RIGHT_WRIST_SERIAL --output recordings/orbbec_wrist_right
```

For live publication, assign distinct collection prefixes such as `orbbec_wrist/left`
and `orbbec_wrist/right`, and root IDs such as `orbbec_wrist_left` and `orbbec_wrist_right`. Use distinct
session recording names for every tracker instance too. One process currently
opens one camera; launching a pair does not synchronize their clocks.

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

Copy [orbbec_wrist.example.toml](orbbec_wrist.example.toml) to a run-specific file and edit its
`[record]` table. It requests a 30-second H.264 capture with IMU enabled. CLI values
override TOML regardless of the position of `--config`; relative paths resolve
from the invoking working directory. Unknown keys, extra tables, and invalid types
are rejected. TOML loading needs Python 3.11+ with `tomllib`.

The helper's `--output DIRECTORY` requires a new directory and creates
`metadata.mcap` plus `raw/Color.*` by default. `--media-mode embedded` stores
video in MCAP; `--keep-media-sidecars` additionally retains raw media in that mode. `--dry-run` prints the native command
without creating an output directory or opening hardware, but still requires
an executable plugin.

## Architecture

```text
Orbbec Wrist Color + Accel/Gyro -> OrbbecSDK -> WristCamera -> FrameSink
                                                       |-> raw/Color.* (default)
                                                       |-> local MCAP: metadata + IMU + diagnostics (+ embedded media)
                                                       `-> SchemaPusher -> generated trackers -> session MCAP
```

The root [main.cpp](main.cpp) and [plugin.yaml](plugin.yaml) follow the repository's
plugin entry-point conventions. Private acquisition, backend, frame-sink, and
metadata-sink code lives in [core/](core/). Vendor headers and handles are confined
to [orbbec_backend.cpp](core/orbbec_backend.cpp); acquisition types do not depend
on SchemaPusher, generated schemas, or MCAP.

FrameSink removes recognized Orbbec timestamp SEI and timestamp-only packets.
H.264/H.265 recording waits for parameter sets and a keyframe at startup, sequence
gaps, and capture-epoch changes; MJPEG passes through. Metadata, media records,
and optional sidecars describe the same accepted bytes. No image decoding or raw
stride inference is performed. Decodability still requires device validation.

Camera operations run on one caller thread. Two synchronous SDK pipelines share
the selected device; requested IMU readiness precedes video startup. SDK frame
handles remain private and outgoing buffers own copied data. A separate publication
worker has a 4096-event queue; a full queue or worker failure stops capture rather
than silently dropping queued events. Close drains and joins the publisher.

Repeated lifecycle calls and partial-start cleanup are handled by the camera
layer. Recovery checks the original serial, VID/PID, firmware, and resolved profile;
a new capture epoch marks restart. Cleanup failures or identity/profile changes
prevent automatic recovery. SDK calls cannot be preempted by application deadlines.

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

Native invocations do not create the helper's requested-profile manifest,
capture log, or verification report. Orbbec Wrist has no audio output or
stereo ColorLeft/ColorRight files.

Sidecar mode stores `raw/Color.<format>`; embedded mode stores media records and
can optionally retain the same sidecars.
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

`calibration.json` contains available structured Color intrinsics, original
alignment/IMU YAML, and availability errors. The local MCAP also stores calibration
JSON in its metadata. Export preserves what the SDK provides; it does not prove
that calibration is complete or accurate.

The tested camera did not expose structured Color intrinsics with either SDK
2.9.0 or 2.10.6, although both raw YAML payloads were readable. Require complete
structured Color calibration and both raw payloads explicitly:

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh verify "$RUN" --require-calibration
```

The tested device is rejected by this strict check. The helper also accepts
`record --require-calibration`.

### Verify a Recording

For a helper delivery, verification runs automatically after capture. To
recheck it, including an embedded recording:

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh verify "$RUN"
```

The verifier checks MCAP CRCs, requested versus recorded profiles, monotonic
device times/sequences, both requested IMU streams, sidecar sizes or embedded
media correlation, and complete video decoding/counts. The report includes
SHA-256 fingerprints and observed sequence gaps. A pass checks file consistency
and decodability; it does not assert lossless delivery or physical synchronization.

Success creates `verification_report.txt`; failures create
`verification_report.failed.txt` and remove a stale success report. Capture
failures retain partial MCAP and logs for diagnosis.

For H.264 captured with a sidecar, decode the complete stream and inspect its
actual dimensions:

```bash
ffmpeg -v error -xerror -f h264 -i "$RUN/raw/Color.h264" -f null -
ffprobe -v error -select_streams v:0 \
  -show_entries stream=codec_name,width,height -of default=noprint_wrappers=1 \
  "$RUN/raw/Color.h264"
```

For H.265 use `-f hevc`; for MJPEG use `-f mjpeg`. Elementary streams are not MP4
files. Check MCAP topics, sample counts, epochs, and discontinuities separately;
decoding alone does not verify IMU completeness or cross-stream synchronization.
The helper runs the integrated verifier automatically; these commands are useful
for manually inspecting the raw stream.

### Export and Play Media

Elementary streams are not MP4 files. Play or remux an H.264 sidecar:

```bash
ffplay -f h264 -framerate 30 "$RUN/raw/Color.h264"
ffmpeg -f h264 -framerate 30 -i "$RUN/raw/Color.h264" -c:v copy "$RUN/Color.mp4"
```

Use a fresh MP4 path. For H.265 use `-f hevc` and `.h265`; for MJPEG use
`-f mjpeg` and transcode with `-c:v libx264 -pix_fmt yuv420p`.
The specified frame rate is a playback assumption, not a reconstruction of device
timestamps or synchronization.

There is no Orbbec Wrist `export-media` helper command. Retain media sidecars
when desktop playback is needed, or read embedded media with the record type
and channel described below.

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

Older combined metadata/media recordings reserve the deprecated `encoded_data`
field, which current metadata accessors omit; extracting those bytes needs a
legacy-schema or reflection reader. Recordings using former unprefixed camera
enum names or `core.WristImuBatchRecord` fail current schema conformance checks.
The former `wrist_camera/Imu/Accel` and `wrist_camera/Imu/Gyro` topic layout needs
its matching reader. No automatic schema or topic migration is provided.

### Timestamps and Synchronization Limits

`device_timestamp_us` preserves the SDK device timestamp;
`system_timestamp_us` preserves its host receipt timestamp, whose OS clock origin
is **UNKNOWN**. `retrieved_monotonic_ns` is Linux `CLOCK_MONOTONIC` at frameset
retrieval, used as a host sample-time proxy. Record wrappers preserve sample time
and checked device-microsecond conversion to nanoseconds; available time is
observed separately. Device time discontinuities are flagged, without claiming a
clock mapping or dropping the original sample at the acquisition layer.

Global timestamp mapping, device clock synchronization, and external synchronization
are not enabled. Exposure timing, IMU axes, drift, cross-stream alignment accuracy,
and SDK buffering/drop behavior remain **UNKNOWN**. Retrieval timestamps and a
left/right process pair do not establish synchronized exposure or sampling.

Hardware tests observed IMU sequence gaps and intermittent device-clock forward
jumps. Some jumps were not marked as discontinuities and passed the integrated
verifier; a verification pass does not rule out these anomalies. A subsequent
ten-second SDK 2.10.6 capture had no detected clock outlier, which does not prove
that the intermittent issue is fixed.

### Development Validation Status

On Linux, with `BUILD_PLUGIN_ORBBEC_WRIST_CAMERA=ON`, `wrist_camera_tests` compiles
production sources with an injected SDK-free backend. Tests cover lifecycle,
partial startup, timeout/cancellation, repeated start/stop, recovery identity,
owned payloads, frame gates, separate metadata/media, MCAP readback, and generated
replay. Related live-tracker tests cover factory discovery and dispatch.

In CMake Tools, enable `BUILD_TESTING=ON`, list targets, and build
`wrist_camera_tests`, `schema_tests`, and `live_trackers_tests`; also build
`schema_py` and `deviceio_trackers_py` when validating Python consumers. List tests
with `ListTests_CMakeTools`, then use `RunCtest_CMakeTools` for Orbbec Wrist, schema
conformance, and relevant live/replay tests. Exercise SDK-off and SDK-on builds
separately; fake-backend tests need no camera or running OpenXR service.

The Python recording suite in [test_recording.py](../../../tests/python/core/orbbec_wrist/test_recording.py)
covers helper defaults/overrides, removed resolution options, media modes, and
verification failures. Hardware acceptance of a new build should cover both IMU
rates, requested codecs, recovery and preview; unit tests do not replace it.

Local validation covered 1,296 script-level combinations and 768 hardware core
cases with SDK 2.9.0:
288 deliveries succeeded and 480 requests were rejected as expected. These counts
do not mean that every build or full parameter combination was exercised.
Real OpenXR/multiple-camera acceptance and long-duration reliability remain pending.

Separate first-use validation covered a fresh SDK 2.10.6 download, checksum
verification, fresh build, library loading,
and ten-second default recording. The original one-command installation stopped
at noninteractive sudo; system dependency and udev installation were skipped
before CMake Tools completed the build. Missing Python `mcap` blocked initial
recording until installed. Color intrinsics and IMU sequence gaps remained.

## Troubleshooting

| Symptom | What to check |
|---|---|
| No device or permission denied | Install the selected SDK's Orbbec Wrist `2bc5:1203` udev rule, reconnect, and run `--list-devices`. Containers also need host USB access. |
| SDK checksum or file validation fails | Retry the installer or select a compatible local SDK; do not bypass integrity checks. |
| SDK library cannot load | Select the same external SDK used for the build and expose its `lib` directory through `--sdk-root` or the native loader path. |
| Requested profile rejected | Check the actual device's video/IMU profiles; defaults are requests. Use `--disable-imu` only for intentional video-only capture. |
| Camera is busy | Stop other processes opening that serial, including SDK viewers. Assign different serials to a wrist pair. |
| Recording waits after disconnect | Reconnect the same camera within `--reconnect-timeout-ms`; firmware/profile changes or cleanup failures prevent recovery. |
| Nonzero exit or `.partial` remains | Preserve capture and logs; check output errors, queue failures, timeout, and hard-link support. |
| Live trackers have no data | Check the OpenXR runtime, generated bindings, plugin search path, exact collection IDs, and enabled IMU. |
| Session MCAP has no encoded video | Use `embedded_recording` to merge committed media fragments, or retain a local plugin MCAP alongside publication; metadata trackers carry no video bytes. |
| Video does not open in a desktop player | Retain a sidecar, decode with FFmpeg, then remux/transcode with the correct format. |
| Unsupported option is rejected | Check the Orbbec Wrist native/help tables for supported options; the plugin provides one Color stream and no audio output. |
| Python MCAP reader is missing | Install `mcap` in the same `python3` environment used by the helper. Native installation does not install it. |
| Strict calibration fails | Inspect `calibration.json`; the tested device lacks structured Color intrinsics even with SDK 2.10.6. |
| Verifier passes but IMU has gaps or clock jumps | Inspect sequence and timestamp continuity separately; delivery integrity is not a zero-loss or synchronization guarantee. |

## Quick Start with orbbec_wrist.sh (Optional)

After installation, use the same external SDK selected for the build:

```bash
./src/plugins/orbbec_wrist/orbbec_wrist.sh doctor --sdk-root "$ORBBEC_SDK_ROOT"
./src/plugins/orbbec_wrist/orbbec_wrist.sh list --sdk-root "$ORBBEC_SDK_ROOT"
./src/plugins/orbbec_wrist/orbbec_wrist.sh record --sdk-root "$ORBBEC_SDK_ROOT" \
  --config src/plugins/orbbec_wrist/orbbec_wrist.example.toml \
  --serial-number YOUR_WRIST_SERIAL --duration 10 --output "recordings/orbbec_wrist_$(date +%Y%m%d_%H%M%S)"
```

The example overrides the TOML duration with a ten-second run. Sidecar media is
saved by default. Use `--no-imu` for deliberate video-only recording, or
`--dry-run` to inspect the command first. `doctor` checks setup, loading, and USB access where available;
`list` reports compatible identities. `record` finalizes the outputs and runs
automatic post-capture verification.

The shell `record` command accepts `--mcap-media-spool PATH` / TOML
`mcap_media_spool` as an additional fragment destination. It still writes and
verifies its normal local delivery; it does not start or merge a TeleopSession.
