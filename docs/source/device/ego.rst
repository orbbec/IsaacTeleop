.. SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
.. SPDX-License-Identifier: Apache-2.0

EGO Camera Plugin
=================

The ``ego_camera`` plugin connects EGO stereo cameras to Isaac Capture.
It records ColorLeft and ColorRight video, and captures accelerometer,
gyroscope, microphone, calibration, and device state when the device exposes
them. Detailed operator commands live in
:code-file:`src/plugins/ego/README.md`.

.. contents:: On this page
   :local:
   :depth: 2

Supported streams and profiles
------------------------------

The plugin captures ``ColorLeft`` and ``ColorRight`` in MJPEG, H.264, or H.265.
The recording wrapper defaults to stereo H.264 ``1600x1300@30``. Use
``capabilities`` to inspect the connected device's available profiles;
unsupported simultaneous combinations fail without a fallback. The current
plugin rejects H.264/H.265 profiles above 30 FPS and does not capture depth or
infrared streams.

Installation
------------

Use an extracted Linux OrbbecSDK v2 package and the normal Isaac Capture source
build dependencies. Install the SDK package's udev rules and reconnect the
camera before using it as a non-root user. The SDK is an external dependency;
the :code-file:`plugin README <src/plugins/ego/README.md>` describes its expected
layout and advanced build options.

From the repository root:

.. code-block:: bash

   export ORBBEC_SDK_ROOT=/path/to/OrbbecSDK
   ./src/plugins/ego/ego.sh doctor --sdk-root "$ORBBEC_SDK_ROOT"
   ./src/plugins/ego/ego.sh build --sdk-root "$ORBBEC_SDK_ROOT" --jobs 8
   ./src/plugins/ego/ego.sh capabilities

Preview is optional and requires SDL2 and FFmpeg development libraries; enable
it with the wrapper's ``build --preview`` flag. Recording alone needs neither.

Capture and verify
------------------

The wrapper records the default stereo profile with available auxiliary
sensors. Give each recording a fresh output directory:

.. code-block:: bash

   RUN="recordings/ego_$(date +%Y%m%d_%H%M%S)"
   ./src/plugins/ego/ego.sh record --duration 30 --output "$RUN"
   ./src/plugins/ego/ego.sh verify "$RUN"

Raw video is an elementary MJPEG, H.264, or H.265 stream. Audio is 48 kHz mono
S16_LE WAV. Metadata, IMU, audio offsets, calibration, and state are recorded
in MCAP. The wrapper also preserves profile and software/device provenance.

For one self-contained MCAP with encoded video and PCM payloads:

.. code-block:: bash

   RUN="recordings/ego_embedded_$(date +%Y%m%d_%H%M%S)"
   ./src/plugins/ego/ego.sh record --duration 30 \
     --output "$RUN" --mcap-media embedded
   ./src/plugins/ego/ego.sh verify "$RUN"
   ./src/plugins/ego/ego.sh export-media "$RUN"

Embedded recording preserves the encoded media without transcoding. Export
produces ordinary video files and WAV. A remaining ``.partial`` file means the
capture did not finish successfully; preserve it and the log for diagnosis.

Data flow and published collections
-----------------------------------

The plugin writes captured media and sensor records and can publish structured
data through SchemaPusher. DeviceIO trackers read the published collections
during session updates.

With ``--collection-prefix=ego``, the collections and Python read APIs are:

.. list-table::
   :header-rows: 1
   :widths: 30 40 30

   * - Collection
     - Python tracker
     - Read method
   * - ``ego/ColorLeft``, ``ego/ColorRight``
     - ``EgoFrameMetadataTracker``
     - ``get_stream_data``
   * - ``ego/Accel``, ``ego/Gyro``
     - ``EgoImuTracker``
     - ``get_stream_data``
   * - ``ego/Audio``
     - ``EgoAudioTracker``
     - ``get_data``
   * - ``ego/Calibration``
     - ``EgoCalibrationTracker``
     - ``get_data``
   * - ``ego/DeviceState``
     - ``EgoDeviceStateTracker``
     - ``get_data``

All reads return the schema payload directly or ``None`` when unavailable.
Camera and IMU readers select one configured stream by its index. Camera streams
use ``EgoCameraStream`` enums; IMU sensors default to Accel and Gyro. See the
:code-file:`plugin README <src/plugins/ego/README.md>` for the full tracker
configuration example.

Multi-device sessions
---------------------

Add the EGO trackers to a normal
:doc:`TeleopSession <../getting_started/teleop_session>` configuration and its
``McapRecordingConfig`` tracker-name list, alongside the hand, head, or
controller trackers. Configure ``ego_camera`` through ``PluginConfig`` with
``--collection-prefix=ego`` and ``shutdown_timeout_seconds=15.0`` to allow
capture cleanup. Live publication requires a running OpenXR runtime.
Complete the normal full source build and install its Python wheel before
using the EGO tracker bindings and recording helper; the wrapper's
``build`` command builds only the native plugin tools.

For one self-contained multi-device MCAP, use
``isaaccapture.ego.embedded_recording`` with exactly one EGO plugin configured
for embedded media. The full configuration, matching media-spool path, and
clean-stop procedure are documented under "TeleopSession: one MCAP with other
trackers" in the :code-file:`plugin README <src/plugins/ego/README.md>`.

Replay and visualization
------------------------

Replay structured records through ReplaySession:

.. code-block:: bash

   uv pip install -e ./examples/mcap_record_replay
   python -m isaaccapture_examples.mcap_record_replay.replay_ego "$RUN/metadata.mcap"

The example reads camera, IMU, audio, calibration, and state records. Use
``export-media`` followed by FFmpeg for encoded media playback. For a live GPU
preview, build the standalone camera_viz binding:

.. code-block:: bash

   examples/camera_viz/camera_viz.sh setup --with-ego \
     --sdk-root "$ORBBEC_SDK_ROOT"
   examples/camera_viz/camera_viz.sh run \
     examples/camera_viz/configs/ego.yaml --mode window --stereo-debug sbs

MJPEG uses SDK conversion and GPU upload; H.264/H.265 uses native NVDEC. Window
SBS is an optional left/right inspection image and is refused in XR mode.
XR mode submits separate eye textures using the existing visualization API.
The viewer and recording plugin each open the camera; run them one at a time.

Troubleshooting
---------------

- **No camera or USB access:** install the selected SDK's udev rules, reconnect,
  and re-run ``doctor`` and ``capabilities``.
- **Profile rejected:** choose an exact advertised left/right combination;
  the current plugin rejects H.264/H.265 profiles above 30 FPS.
- **Partial MCAP:** the recording did not finish successfully. Preserve it and
  the log for diagnosis before starting a new recording.
- **Publisher startup failure:** check the running OpenXR runtime and plugin
  search path.
- **No embedded final output:** check normal context exit, committed fragment,
  matching spool path, and the merge tool beside the selected plugin.
- **Native decoder unavailable:** build camera_viz's codec with CUDA, or choose
  an advertised MJPEG profile for that preview.
