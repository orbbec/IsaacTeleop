# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Replay Color frame metadata and independent Accel/Gyro batches from Wrist MCAP.

Accepts a local capture.mcap or a finalized TeleopSession archive. Encoded video
and JSON diagnostics/calibration are not replayed by these structured trackers.
"""

import argparse
import logging
from collections import Counter
from pathlib import Path

from isaaccapture.deviceio_session import McapReplayConfig, ReplaySession
from isaaccapture.deviceio_trackers import (
    OrbbecWristFrameMetadataTracker,
    OrbbecWristImuTracker,
)
from mcap.reader import make_reader

logger = logging.getLogger(
    "isaaccapture.examples.mcap_record_replay.replay_orbbec_wrist"
)


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


def replay(
    path: Path,
    max_updates: int = 0,
    *,
    topic_prefix: str = "wrist_camera",
    include_imu: bool = True,
) -> dict[str, int]:
    """Replay one independently recorded camera; return payload counts per stream."""
    topic_prefix = topic_prefix.rstrip("/")
    if not topic_prefix:
        raise ValueError("topic_prefix must not be empty")
    if max_updates < 0:
        raise ValueError("max_updates must be non-negative")

    frame_tracker = OrbbecWristFrameMetadataTracker("wrist_metadata")
    trackers = [(frame_tracker, topic_prefix)]
    imu_trackers = {}
    topics = {f"{topic_prefix}/Color"}
    if include_imu:
        for name in ("Accel", "Gyro"):
            tracker = OrbbecWristImuTracker(f"wrist_imu/{name}")
            imu_trackers[name] = tracker
            trackers.append((tracker, f"{topic_prefix}/{name}"))
            topics.add(f"{topic_prefix}/{name}/Imu")

    counts = dict.fromkeys(("Color", *imu_trackers), 0)
    # Each update consumes one record per stream; None can be a payload-less record.
    updates = _update_count(path, topics)
    if max_updates:
        updates = min(updates, max_updates)
    if not updates:
        return counts

    with ReplaySession.run(McapReplayConfig(str(path), trackers)) as session:
        for _ in range(updates):
            session.update()
            frame = frame_tracker.get_data(session)
            if frame is not None:
                counts["Color"] += 1
                print(
                    f"Color: epoch={frame.capture_epoch} seq={frame.sequence_number} "
                    f"profile={frame.width}x{frame.height}@{frame.fps} "
                    f"format={frame.pixel_format} bytes={frame.encoded_bytes}"
                )
            for name, tracker in imu_trackers.items():
                batch = tracker.get_data(session)
                if batch is not None:
                    counts[name] += 1
                    print(
                        f"{name}: epoch={batch.capture_epoch} "
                        f"rate_hz={batch.sample_rate_hz} samples={len(batch.samples)}"
                    )
    return counts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap", type=Path, help="Wrist capture.mcap or session archive")
    parser.add_argument(
        "--topic-prefix",
        default="wrist_camera",
        help="Camera's MCAP topic prefix (default: wrist_camera)",
    )
    parser.add_argument(
        "--no-imu", action="store_true", help="Replay only Color frame metadata"
    )
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
    if not args.topic_prefix.rstrip("/"):
        parser.error("--topic-prefix must not be empty")

    counts = replay(
        args.mcap,
        args.max_updates,
        topic_prefix=args.topic_prefix,
        include_imu=not args.no_imu,
    )
    print(
        "Replay counts: "
        + ", ".join(f"{name}={count}" for name, count in counts.items())
    )
    missing = ", ".join(name for name, count in counts.items() if count == 0)
    if missing:
        logger.error("Replay incomplete; no payloads for: %s", missing)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
