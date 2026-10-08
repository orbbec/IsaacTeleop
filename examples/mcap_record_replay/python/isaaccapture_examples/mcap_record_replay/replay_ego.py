# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Replay all structured EGO trackers from a recorded MCAP file.

Pass the ``metadata.mcap`` produced by ``ego.sh record``, or the
final archive produced by ``isaaccapture.ego.embedded_recording``.
"""

import argparse
import logging
from collections import Counter
from pathlib import Path

from isaaccapture.deviceio_session import McapReplayConfig, ReplaySession
from isaaccapture.deviceio_trackers import (
    EgoFrameMetadataTracker,
    EgoAudioTracker,
    EgoCalibrationTracker,
    EgoDeviceStateTracker,
    EgoImuTracker,
)
from isaaccapture.schema import EgoCameraStream
from mcap.reader import make_reader

logger = logging.getLogger("isaaccapture.examples.mcap_record_replay.replay_ego")


def _update_count(path: Path, topics: set[str]) -> int:
    with path.open("rb") as stream:
        reader = make_reader(stream)
        summary = reader.get_summary()
        counts: Counter[str] = Counter()
        if (
            summary is not None
            and summary.statistics is not None
            and summary.statistics.channel_message_counts.keys()
            <= summary.channels.keys()
        ):
            for channel_id, count in summary.statistics.channel_message_counts.items():
                topic = summary.channels[channel_id].topic
                if topic in topics:
                    counts[topic] += count
        else:
            for _, channel, _ in reader.iter_messages(
                topics=topics, log_time_order=False
            ):
                counts[channel.topic] += 1
    return max(counts.values(), default=0)


def replay(path: Path, max_updates: int) -> dict[str, int]:
    frame_tracker = EgoFrameMetadataTracker(
        "ego_metadata",
        [EgoCameraStream.ColorLeft, EgoCameraStream.ColorRight],
    )
    imu_tracker = EgoImuTracker("ego_imu")
    audio_tracker = EgoAudioTracker("ego_audio/Audio")
    calibration_tracker = EgoCalibrationTracker("ego_calibration/Calibration")
    state_tracker = EgoDeviceStateTracker("ego_device/DeviceState")
    trackers = [
        (frame_tracker, "ego_metadata"),
        (imu_tracker, "ego_imu"),
        (audio_tracker, "ego_audio"),
        (calibration_tracker, "ego_calibration"),
        (state_tracker, "ego_device"),
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
    topics = {
        "ego_metadata/ColorLeft",
        "ego_metadata/ColorRight",
        "ego_imu/Accel",
        "ego_imu/Gyro",
        "ego_audio/Audio",
        "ego_calibration/Calibration",
        "ego_device/DeviceState",
    }
    # ReplaySession has no EOF accessor; payload-less records also return None.
    updates = _update_count(path, topics)
    if max_updates:
        updates = min(updates, max_updates)

    with ReplaySession.run(McapReplayConfig(str(path), trackers)) as session:
        for _ in range(updates):
            session.update()

            for index, name in enumerate(("ColorLeft", "ColorRight")):
                data = frame_tracker.get_stream_data(session, index)
                if data is not None:
                    counts[name] += 1
                    print(
                        f"{name}: epoch={data.capture_epoch} seq={data.sequence_number} "
                        f"profile={data.width}x{data.height}@{data.fps} "
                        f"format={data.pixel_format} bytes={data.encoded_bytes}"
                    )

            for index, name in enumerate(("Accel", "Gyro")):
                data = imu_tracker.get_stream_data(session, index)
                if data is not None:
                    counts[name] += 1
                    print(
                        f"{name}: epoch={data.capture_epoch} seq={data.sequence_number} "
                        f"rate_hz={data.sample_rate_hz} samples={len(data.samples)}"
                    )

            audio = audio_tracker.get_data(session)
            if audio is not None:
                counts["Audio"] += 1
                print(
                    f"Audio: epoch={audio.capture_epoch} seq={audio.sequence_number} "
                    f"rate_hz={audio.sample_rate_hz} samples={audio.sample_count} "
                    f"bytes={audio.byte_count}"
                )

            calibration = calibration_tracker.get_data(session)
            if calibration is not None:
                counts["Calibration"] += 1
                print(
                    f"Calibration: epoch={calibration.capture_epoch} "
                    f"device_uid={calibration.device_uid}"
                )

            state = state_tracker.get_data(session)
            if state is not None:
                counts["DeviceState"] += 1
                print(
                    f"DeviceState: epoch={state.capture_epoch} seq={state.sequence_number} "
                    f"connection={state.connection_state} reconnect_attempt={state.reconnect_attempt} "
                    f"device_uid={state.device_uid}"
                )

    return counts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap", type=Path, help="Path to an EGO metadata.mcap file")
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
        logger.error("Replay incomplete; no records for: %s", missing)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
