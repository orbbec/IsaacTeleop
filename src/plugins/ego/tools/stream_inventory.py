# SPDX-FileCopyrightText: Copyright (c) 2026 ORBBEC INC. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Canonical video names shared by the standalone EGO verification tools."""

import json
from pathlib import Path
import sys

VIDEO_STREAMS = ("ColorLeft", "ColorRight", "ColorSideLeft", "ColorSideRight")
STREAM_IDS = {name: index for index, name in enumerate(VIDEO_STREAMS)}


def metadata_topic(stream):
    return f"orbbec_ego_metadata_{stream.lower()}/FrameMetadata"


# Follow the current upstream layout; legacy recording conversion is out of scope.
VIDEO_METADATA_STREAMS = {metadata_topic(name): name for name in VIDEO_STREAMS}


def requested_streams(path):
    path = Path(path)
    if not path.exists():
        return None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SystemExit(
            f"Invalid requested profile manifest {path}: {error}"
        ) from error
    streams = value.get("streams") if isinstance(value, dict) else None
    if (
        not isinstance(value, dict)
        or value.get("version") != 1
        or not isinstance(streams, dict)
        or not streams
        or any(name not in STREAM_IDS for name in streams)
    ):
        raise SystemExit(f"Invalid requested video stream inventory in {path}")
    return tuple(name for name in VIDEO_STREAMS if name in streams)


def recorded_streams(path):
    from mcap.reader import make_reader

    with Path(path).open("rb") as stream:
        reader = make_reader(stream)
        summary = reader.get_summary()
        topics = (
            {channel.topic for channel in summary.channels.values()}
            if summary is not None
            else {
                channel.topic
                for _, channel, _ in reader.iter_messages(log_time_order=False)
            }
        )
    metadata_topics = {
        topic
        for topic in topics
        if topic.startswith("orbbec_ego_metadata_") and topic.endswith("/FrameMetadata")
    }
    unknown = metadata_topics - VIDEO_METADATA_STREAMS.keys()
    if unknown:
        raise SystemExit(f"Unknown EGO video topics: {sorted(unknown)}")
    names = {VIDEO_METADATA_STREAMS[topic] for topic in metadata_topics}
    if not names:
        raise SystemExit("MCAP contains no EGO video metadata channels")
    media = {
        topic.removeprefix("orbbec_ego_media/")
        for topic in topics
        if topic.startswith("orbbec_ego_media/") and topic != "orbbec_ego_media/Audio"
    }
    if media and media != names:
        raise SystemExit(
            f"Embedded video streams {sorted(media)} differ from metadata {sorted(names)}"
        )
    return tuple(name for name in VIDEO_STREAMS if name in names)


def video_streams(path, manifest):
    actual = recorded_streams(path)
    requested = requested_streams(manifest)
    if requested is not None and set(actual) != set(requested):
        raise SystemExit(
            f"Recorded video streams {list(actual)} differ from requested {list(requested)}"
        )
    return requested or actual


if __name__ == "__main__":
    print("\n".join(video_streams(sys.argv[1], sys.argv[2])))
