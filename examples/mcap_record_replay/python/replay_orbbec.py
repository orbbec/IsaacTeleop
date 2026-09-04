# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Replay every structured Orbbec tracker from a recorded MCAP file.

This example is headless and reads the existing Isaac Teleop MCAP schemas; it
does not implement another MCAP reader. Pass the ``metadata.mcap`` produced by
``src/plugins/orbbec/orbbec_ego.sh record``.

Usage:
    python replay_orbbec.py path/to/metadata.mcap
"""

import argparse
from pathlib import Path

from isaacteleop.deviceio_session import McapReplayConfig, ReplaySession
from isaacteleop.deviceio_trackers import (
    FrameMetadataTrackerOrbbec,
    OrbbecAudioTracker,
    OrbbecCalibrationTracker,
    OrbbecDeviceStateTracker,
    OrbbecImuTracker,
)
from isaacteleop.schema import OrbbecCameraStream


def replay(path: Path, max_updates: int) -> dict[str, int]:
    frame_tracker = FrameMetadataTrackerOrbbec(
        "orbbec_metadata",
        [OrbbecCameraStream.ColorLeft, OrbbecCameraStream.ColorRight],
    )
    imu_tracker = OrbbecImuTracker("orbbec_imu")
    audio_tracker = OrbbecAudioTracker("orbbec_audio")
    calibration_tracker = OrbbecCalibrationTracker("orbbec_calibration")
    state_tracker = OrbbecDeviceStateTracker("orbbec_device")
    trackers = [
        (frame_tracker, "orbbec_metadata"),
        (imu_tracker, "orbbec_imu"),
        (audio_tracker, "orbbec_audio"),
        (calibration_tracker, "orbbec_calibration"),
        (state_tracker, "orbbec_device"),
    ]
    counts = {
        "ColorLeft": 0,
        "ColorRight": 0,
        "Accel": 0,
        "Gyro": 0,
        "Audio": 0,
        "Calibration": 0,
        "DeviceState": 0,
    }

    with ReplaySession.run(McapReplayConfig(str(path), trackers)) as session:
        update = 0
        while max_updates == 0 or update < max_updates:
            session.update()
            update += 1
            present = 0

            for index, name in enumerate(("ColorLeft", "ColorRight")):
                data = frame_tracker.get_stream_data(session, index).data
                if data is not None:
                    present += 1
                    counts[name] += 1
                    print(
                        f"{name}: epoch={data.capture_epoch} seq={data.sequence_number} "
                        f"profile={data.width}x{data.height}@{data.fps} "
                        f"format={data.pixel_format} bytes={data.encoded_bytes}"
                    )

            for index, name in enumerate(("Accel", "Gyro")):
                data = imu_tracker.get_stream_data(session, index).data
                if data is not None:
                    present += 1
                    counts[name] += 1
                    print(
                        f"{name}: epoch={data.capture_epoch} seq={data.sequence_number} "
                        f"rate_hz={data.sample_rate_hz} samples={len(data.samples)}"
                    )

            audio = audio_tracker.get_data(session).data
            if audio is not None:
                present += 1
                counts["Audio"] += 1
                print(
                    f"Audio: epoch={audio.capture_epoch} seq={audio.sequence_number} "
                    f"rate_hz={audio.sample_rate_hz} samples={audio.sample_count} "
                    f"bytes={audio.byte_count}"
                )

            calibration = calibration_tracker.get_data(session).data
            if calibration is not None:
                present += 1
                counts["Calibration"] += 1
                print(
                    f"Calibration: epoch={calibration.capture_epoch} "
                    f"device_uid={calibration.device_uid}"
                )

            state = state_tracker.get_data(session).data
            if state is not None:
                present += 1
                counts["DeviceState"] += 1
                print(
                    f"DeviceState: epoch={state.capture_epoch} seq={state.sequence_number} "
                    f"connection={state.connection_state} reconnect_attempt={state.reconnect_attempt} "
                    f"device_uid={state.device_uid}"
                )

            if present == 0:
                break

    return counts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap", type=Path, help="Path to an Orbbec metadata.mcap file")
    parser.add_argument(
        "--max-updates",
        type=int,
        default=0,
        help="Stop after this many replay updates; 0 reads to end of file",
    )
    args = parser.parse_args()
    if not args.mcap.is_file():
        parser.error(f"MCAP file does not exist: {args.mcap}")
    if args.max_updates < 0:
        parser.error("--max-updates must be non-negative")

    counts = replay(args.mcap, args.max_updates)
    print(
        "Replay counts: "
        + ", ".join(f"{name}={count}" for name, count in counts.items())
    )
    if not all(counts.values()):
        missing = ", ".join(name for name, count in counts.items() if count == 0)
        print(f"Replay incomplete; no records for: {missing}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
