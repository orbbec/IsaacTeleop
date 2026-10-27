.. SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
.. SPDX-License-Identifier: Apache-2.0

Orbbec Wrist Camera Plugin
==========================

The Orbbec Wrist plugin connects a wrist-mounted Orbbec camera to Isaac Capture.
Each process captures one encoded Color stream and optional accelerometer and
gyroscope data. It supports standalone recording and live metadata publication
to Isaac Teleop sessions through DeviceIO.

Run shell examples from the repository root. The
:code-file:`plugin README <src/plugins/orbbec_wrist/README.md>` documents the
complete command reference and configuration examples.

.. contents:: On this page
   :local:
   :depth: 2

Supported hardware and data
---------------------------

The plugin selects compatible Orbbec Wrist devices with USB VID/PID
``2bc5:1203``. One compatible camera is selected automatically; specify
``--serial-number`` when several are connected. Recovery remains pinned to the
selected physical camera.

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Stream
     - Supported configuration
   * - Color
     - One 1600 × 1200 stream, H.264, H.265, or MJPEG; default H.264 at 30 FPS.
   * - Accelerometer
     - Optional, enabled by default; 400 or 1000 Hz; default range 24 g.
   * - Gyroscope
     - Optional, enabled by default; same requested rate as the accelerometer,
       with a 2000 dps range.

Audio, depth, IR, and raw image output are not provided. Profiles must be
advertised by the connected device; unsupported requests fail without profile
fallback. Width and height overrides are rejected. The accepted 16 g request
also requires device support.

Installation
------------

On Linux x86_64, install CMake 3.24 or newer and ``uv``, then run:

.. code-block:: bash

   ./src/plugins/orbbec_wrist/install.sh --jobs 8

The installer prepares native dependencies, downloads the pinned Orbbec SDK,
verifies its SHA-256, installs USB udev rules, and builds the executable. Run as
your normal user in an interactive terminal; missing system packages and host
udev setup require sudo access. Reconnect the camera after installing the rules.
The initial build also downloads source dependencies, so it requires network
access beyond the SDK download.

The current installer selects SDK 2.10.6 from the vendor's
`pinned release <https://github.com/orbbec/OrbbecSDK_v2-EGO/releases/tag/v2.10.6-EGO-20261019>`_.
To use an existing extracted package:

.. code-block:: bash

   export ORBBEC_SDK_ROOT=/absolute/path/to/OrbbecSDK
   ./src/plugins/orbbec_wrist/install.sh --sdk-root "$ORBBEC_SDK_ROOT" --jobs 8

Keep the external SDK's libraries and extensions together. All Orbbec camera
plugins enabled in one build tree must use the same SDK root. Containers need
USB access and permissions configured on the host. Prepared hosts can use
``--skip-deps`` and ``--skip-udev``; these options assume that setup is complete.

The native build helper can rebuild an existing SDK configuration:

.. code-block:: bash

   ./src/plugins/orbbec_wrist/orbbec_wrist.sh build \
     --sdk-root "$ORBBEC_SDK_ROOT" --preset py3.11 --jobs 8

This builds the executable in ``build-wrist-py3.11``. It does not build or
install the Python package and generated tracker bindings. Complete the
:doc:`source build <../../development/build_from_source/index>` before using
the Python session workflow below. The relevant CMake options are
``BUILD_PLUGINS=ON``, ``BUILD_PLUGIN_ORBBEC_WRIST_CAMERA=ON``, and
``ORBBEC_SDK_ROOT``. Optional native preview uses
``ORBBEC_WRIST_ENABLE_PREVIEW=ON``.

Recording verification also requires FFmpeg, FFprobe, Python 3, and the Python
``mcap`` package. The installer does not install this Python dependency. In an
activated virtual environment, install it into the interpreter used by the
helper:

.. code-block:: bash

   uv pip install --python "$(command -v python3)" 'mcap>=1.2,<2'

For a session environment containing the built Isaac Capture package, its
``orbbec-wrist`` extra declares this dependency.

Discover, record, and verify
----------------------------

After the default SDK download, select that package and check setup:

.. code-block:: bash

   export ORBBEC_SDK_ROOT="$PWD/build/wrist-sdk/v2.10.6-EGO-20261019/OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64"
   ./src/plugins/orbbec_wrist/orbbec_wrist.sh doctor --sdk-root "$ORBBEC_SDK_ROOT"
   ./src/plugins/orbbec_wrist/orbbec_wrist.sh list --sdk-root "$ORBBEC_SDK_ROOT"

``doctor`` checks setup and library loading; it does not prove successful
streaming. To inspect advertised profiles, run the native executable:

.. code-block:: bash

   export ORBBEC_WRIST_PLUGIN="$PWD/build-wrist-py3.11/src/plugins/orbbec_wrist/orbbec_wrist_camera_plugin"
   export LD_LIBRARY_PATH="$ORBBEC_SDK_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
   "$ORBBEC_WRIST_PLUGIN" --list-capabilities

Record ten seconds using the default Color and IMU configuration:

.. code-block:: bash

   RUN="recordings/orbbec_wrist_$(date +%Y%m%d_%H%M%S)"
   ./src/plugins/orbbec_wrist/orbbec_wrist.sh record \
     --sdk-root "$ORBBEC_SDK_ROOT" --duration 10 --output "$RUN"
   ./src/plugins/orbbec_wrist/orbbec_wrist.sh verify "$RUN"

The output directory must be new. Add ``--serial-number YOUR_SERIAL`` to select
a device explicitly, ``--imu-rate 1000`` for a supported higher IMU rate, or
``--no-imu`` for video-only recording. Without ``--duration``, recording runs
until Ctrl-C. Press Ctrl-C once and allow cleanup and verification to finish.

The default delivery contains:

.. code-block:: text

   raw/Color.h264
   metadata.mcap
   calibration.json
   capabilities.txt
   requested_profile.json
   logs/capture.log
   verification_report.txt

H.265 and MJPEG use ``Color.h265`` and ``Color.mjpg`` respectively. The helper
automatically verifies CRCs, requested profiles, video decoding, media sizes,
and metadata/IMU continuity. Only successful verification creates
``verification_report.txt``; failures create ``verification_report.failed.txt``.
Keep partial output and logs when capture fails.

To put encoded Color video inside ``metadata.mcap``, add
``--media-mode embedded`` to ``record``. Add ``--keep-media-sidecars`` to retain
the elementary video file too. Verification compares embedded and retained
media when both are present. Media is stored without transcoding.

The helper also accepts :code-file:`src/plugins/orbbec_wrist/orbbec_wrist.example.toml`
through ``record --config PATH``. CLI arguments override TOML values. Helper
options use ``--option value``; native options use ``--option=value``.

Data flow and published collections
-----------------------------------

Standalone recording writes encoded video and structured records locally
without an OpenXR runtime. For live sessions, ``--collection-prefix`` publishes
frame metadata and IMU batches through OpenXR SchemaPusher. DeviceIO trackers
read these collections during session updates. Local MCAP recording and live
publication can be enabled together.

With ``--collection-prefix=orbbec_wrist/left``, use:

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Collection
     - Python tracker
   * - ``orbbec_wrist/left/Color``
     - ``OrbbecWristFrameMetadataTracker``
   * - ``orbbec_wrist/left/Accel``
     - ``OrbbecWristImuTracker``
   * - ``orbbec_wrist/left/Gyro``
     - ``OrbbecWristImuTracker``

Construct each tracker with its collection ID and call ``get_data(session)``
after ``session.step()``. Python returns the schema payload or ``None`` when
unavailable. Frame metadata contains no encoded video bytes. A session recording
of these trackers alone therefore does not include the video.

Multi-device session recording
------------------------------

Live publication requires a running Isaac Capture OpenXR runtime and the built
Python bindings. For an existing live
:doc:`TeleopSession <../../teleop/teleop_session>` configuration named ``config``,
add the camera alongside its other devices:

.. code-block:: python

   from pathlib import Path

   import isaaccapture.deviceio_trackers as dt
   from isaaccapture.deviceio_session import McapRecordingConfig
   from isaaccapture.orbbec_wrist import embedded_recording
   from isaaccapture.teleop_session_manager import PluginConfig

   run = Path("recordings/orbbec_wrist_session").resolve()
   run.mkdir(parents=True, exist_ok=True)
   entries = [
       (dt.OrbbecWristFrameMetadataTracker("orbbec_wrist/left/Color"), "orbbec_wrist/left"),
       (dt.OrbbecWristImuTracker("orbbec_wrist/left/Accel"), "orbbec_wrist/left/Accel"),
       (dt.OrbbecWristImuTracker("orbbec_wrist/left/Gyro"), "orbbec_wrist/left/Gyro"),
   ]
   existing = config.mcap_config.get_tracker_names() if config.mcap_config else []
   config.trackers.extend(tracker for tracker, _ in entries)
   config.mcap_config = McapRecordingConfig(str(run / "final.mcap"), existing + entries)
   config.plugins.append(PluginConfig(
       plugin_name="orbbec_wrist_camera",
       plugin_root_id="orbbec_wrist_left",
       search_paths=[Path("build-wrist-py3.11/src/plugins").resolve()],
       required=True,
       shutdown_timeout_seconds=5.0,
       plugin_args=[
           "--serial-number=YOUR_SERIAL",
           "--collection-prefix=orbbec_wrist/left",
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

The helper adds ``--media-mode=embedded`` and ``--mcap-media-spool=PATH`` to the
plugin configuration. After clean shutdown, it merges ``final.session.mcap``
and the camera fragment into ``final.mcap``, preserving the encoded video,
capture metadata, IMU, calibration, and other session trackers. Inputs remain
available. Existing final, intermediate, fragment, or matching ``.partial``
paths are rejected; missing or unclean fragments prevent finalization.

The example sets a 5-second shutdown grace period for this Wrist instance; the
framework default remains 2 seconds. This allows time for camera shutdown and
recording cleanup and requires the configurable shutdown timeout API. Forced
termination can leave an uncommitted ``.partial`` fragment and prevent finalization.

For a second camera, add another plugin and tracker group with a distinct
serial, root ID, collection prefix, and recording names. Map both root IDs to
separate fragment paths. One process opens one camera. This does not synchronize
the two cameras.

Availability monitoring
-----------------------

The plugin participates in
:doc:`device monitoring <../../references/device_provider_monitoring>`.
After a session update, query
``session.get_device_status("orbbec_wrist_left/camera/wrist")`` using the root
ID in the example. The ``/camera/wrist`` suffix is the device path declared by
the plugin manifest.

Streaming maps to ``CONNECTED``; startup and recovery to ``DEGRADED``; capture
faults to ``FAILED``; and closure to ``DISCONNECTED``. Status delivery depends
on the runtime and consumer polling. Process health remains available if device
status publication is unavailable.

Live preview
------------

For an SDL preview during recording, enable optional preview dependencies and
build support, then request a window:

.. code-block:: bash

   ./src/plugins/orbbec_wrist/install.sh --sdk-root "$ORBBEC_SDK_ROOT" --preview --jobs 8
   ./src/plugins/orbbec_wrist/orbbec_wrist.sh record \
     --sdk-root "$ORBBEC_SDK_ROOT" --duration 10 \
     --output "recordings/orbbec_wrist_preview_$(date +%Y%m%d_%H%M%S)" --preview

This requires a graphical desktop. Esc, Q, or closing the SDL window stops
capture and finalizes output. Decoder or window failures disable preview while
recording continues.

For the separate GPU camera_viz viewer:

.. code-block:: bash

   examples/camera_viz/camera_viz.sh setup --with-orbbec-wrist \
     --sdk-root "$ORBBEC_SDK_ROOT"
   examples/camera_viz/camera_viz.sh run \
     examples/camera_viz/configs/orbbec_wrist.yaml --mode window

Set ``serial_number`` in
:code-file:`examples/camera_viz/configs/orbbec_wrist.yaml` when several cameras
are connected. The viewer and plugin each open the camera; run them one at a
time. See :code-file:`examples/camera_viz/README.md` for GPU and decoder setup.
Close the viewer window or use Ctrl-C to exit.

Timestamp and calibration limits
--------------------------------

Device timestamps and host arrival times are preserved separately. Session
polling times and MCAP log times do not establish physical camera/IMU
synchronization. IMU sequence gaps and intermittent device-clock forward jumps
can occur. Discontinuity flags and delivery verification do not detect every
clock anomaly. Inspect sequence and timestamp continuity separately; a
verification pass does not establish zero loss or synchronized sampling.

Calibration export preserves SDK fields and raw YAML with availability errors.
Structured Color intrinsics may be unavailable even when raw YAML is readable.
Use ``verify "$RUN" --require-calibration`` when complete structured Color
calibration and raw YAML are required; this check fails if required fields are missing.
Do not treat the presence of ``calibration.json`` as proof of complete calibration.

Troubleshooting
---------------

- **No device or USB access:** install the selected SDK's udev rules, reconnect,
  run ``doctor`` and ``list``, and check USB enumeration. A successful ``list``
  exit alone does not prove that a camera was found.
- **SDK library loading fails:** use the SDK selected at build time, keep its
  extensions directory intact, and pass ``--sdk-root`` to the helper.
- **Python MCAP import fails:** install ``mcap`` into the helper's active
  ``python3`` environment before recording.
- **Profile rejected:** inspect native ``--list-capabilities`` output and use
  an advertised codec, FPS, IMU rate, and range.
- **Output already exists:** select a fresh recording directory or session
  output paths; retain failed captures for diagnosis.
- **No video in a session MCAP:** metadata trackers carry no video bytes. Use
  ``embedded_recording`` and retain its committed media fragments.
- **Live publisher startup fails:** check the OpenXR runtime, built tracker
  bindings, plugin search path, and matching collection IDs.
- **Calibration verification fails:** inspect availability errors in the
  exported JSON; missing SDK intrinsics require device/SDK investigation.
