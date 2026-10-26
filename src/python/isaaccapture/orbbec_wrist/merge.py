# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Merge committed Orbbec Wrist fragments without decoding or retiming media."""

from contextlib import ExitStack
import heapq
from collections import Counter
import json
import os
from pathlib import Path
from typing import Iterable

_OBSERVED = "session_observed/"
_STRUCTURED = {
    "core.OrbbecWristFrameMetadataRecord",
    "core.OrbbecWristImuBatchRecord",
}


def _fresh_paths(paths):
    resolved = [Path(path).resolve() for path in paths]
    if len(set(resolved)) != len(resolved):
        raise ValueError("Orbbec Wrist recording paths must be distinct")
    for path in paths:
        path = Path(path)
        if path.exists() or path.is_symlink():
            raise FileExistsError(f"Refusing to overwrite Orbbec Wrist output: {path}")


def _partial(path):
    return Path(str(path) + ".partial")


class _Source:
    def __init__(self, stack, path, *, fragment=False):
        from mcap.reader import make_reader
        from mcap.records import (
            Attachment,
            Channel,
            DataEnd,
            Footer,
            Header,
            Message,
            Metadata,
            Schema,
        )
        from mcap.stream_reader import StreamReader

        self.path = Path(path).resolve()
        if _partial(self.path).exists() or _partial(self.path).is_symlink():
            raise ValueError(f"Uncommitted MCAP input: {self.path}")
        self.stream = stack.enter_context(self.path.open("rb"))
        self.reader = make_reader(self.stream, validate_crcs=True)
        self.summary = self.reader.get_summary()
        if self.summary is None or self.summary.statistics is None:
            raise ValueError(f"MCAP input requires a complete summary: {self.path}")
        self.stream.seek(0)
        self.counts = [0, 0, 0]
        headers = footers = ends = 0
        self.capture = None
        message_counts = Counter()
        last_status = {}
        for item in StreamReader(self.stream, validate_crcs=True).records:
            if isinstance(item, Schema):
                if self.summary.schemas.get(item.id) != item:
                    raise ValueError("MCAP data and summary schemas disagree")
            elif isinstance(item, Channel):
                if self.summary.channels.get(item.id) != item:
                    raise ValueError("MCAP data and summary channels disagree")
            elif isinstance(item, Message):
                self.counts[0] += 1
                message_counts[item.channel_id] += 1
                channel = self.summary.channels[item.channel_id]
                if fragment and channel.topic.endswith("/Diagnostics"):
                    last_status[channel.topic] = json.loads(item.data)
            elif isinstance(item, Metadata):
                self.counts[1] += 1
                if fragment and item.name == "wrist.capture":
                    if self.capture is not None:
                        raise ValueError("Ambiguous Orbbec Wrist capture identity")
                    self.capture = item.metadata
            elif isinstance(item, Attachment):
                self.counts[2] += 1
            elif isinstance(item, Header):
                headers += 1
            elif isinstance(item, Footer):
                footers += 1
            elif isinstance(item, DataEnd):
                ends += 1
        stats = self.summary.statistics
        if (headers, footers, ends) != (1, 1, 1) or self.counts != [
            stats.message_count,
            stats.metadata_count,
            stats.attachment_count,
        ]:
            raise ValueError(f"MCAP inventory disagrees with its summary: {self.path}")
        if any(
            channel.topic.startswith(_OBSERVED)
            for channel in self.summary.channels.values()
        ):
            raise ValueError("Reserved session_observed/ topic already exists")
        topics = {}
        for channel in self.summary.channels.values():
            signature = self.signature(channel)
            if channel.topic in topics and topics[channel.topic] != signature:
                raise ValueError(f"Conflicting schemas for MCAP topic: {channel.topic}")
            topics[channel.topic] = signature
        self.prefix = ""
        if fragment:
            capture = self.capture or {}
            self.prefix = capture.get("collection_prefix", "")
            if (
                capture.get("fragment_version") != "1"
                or capture.get("media_mode") != "embedded"
                or not self.prefix
                or not capture.get("serial_number")
            ):
                raise ValueError(
                    "Expected a version 1 embedded Orbbec Wrist fragment with device identity"
                )
            channels = list(self.summary.channels.values())
            if any(
                not channel.topic.startswith(self.prefix + "/") for channel in channels
            ):
                raise ValueError(
                    "Fragment contains channels outside its collection prefix"
                )
            if not any(
                self.schema(channel)
                and self.schema(channel).name
                == "core.OrbbecWristEncodedVideoFrameRecord"
                and message_counts[channel.id] > 0
                for channel in channels
            ):
                raise ValueError("Fragment has no Orbbec Wrist encoded-video messages")
            terminal = last_status.get(self.prefix + "/Diagnostics", {})
            if terminal.get("state") != 0 or terminal.get("error"):
                raise ValueError("Orbbec Wrist fragment has no clean Closed event")
        self.channels = {}

    def schema(self, channel):
        return self.summary.schemas[channel.schema_id] if channel.schema_id else None

    def signature(self, channel):
        schema = self.schema(channel)
        return (
            channel.message_encoding,
            None if schema is None else (schema.name, schema.encoding, schema.data),
        )


def merge_recording(recording, media_fragments: Iterable[os.PathLike | str], output):
    """Merge one session and one or more Orbbec Wrist fragments; preserve inputs on failure.

    Capture records retain their topics. Overlapping session observations move
    under session_observed/. All payloads, timestamps and sequences are retained.
    """
    from mcap.writer import CompressionType, Writer

    recording, output = Path(recording), Path(output)
    fragments = [Path(path) for path in media_fragments]
    if not fragments:
        raise ValueError("At least one Orbbec Wrist fragment is required")
    inputs = [recording, *fragments]
    identities = [path.resolve() for path in [*inputs, output, _partial(output)]]
    if len(set(identities)) != len(identities):
        raise ValueError("MCAP inputs, output and partial paths must be distinct")
    _fresh_paths([output, _partial(output)])
    if not output.parent.is_dir():
        raise FileNotFoundError(
            f"Create the MCAP output directory first: {output.parent}"
        )
    with ExitStack() as stack:
        host = _Source(stack, recording)
        captures = [_Source(stack, path, fragment=True) for path in fragments]
        for field in ("collection_prefix", "plugin_root_id", "serial_number"):
            values = [source.capture.get(field, "") for source in captures]
            if field == "plugin_root_id":
                values = [value for value in values if value]
            if len(values) != len(set(values)):
                raise ValueError(f"Duplicate Orbbec Wrist {field}")
        capture_topics = {}
        for source in captures:
            for channel in source.summary.channels.values():
                if channel.topic in capture_topics:
                    raise ValueError(f"Conflicting fragment topic: {channel.topic}")
                capture_topics[channel.topic] = (source, channel)
        observed = set()
        for channel in host.summary.channels.values():
            if channel.topic not in capture_topics:
                continue
            source, capture = capture_topics[channel.topic]
            schema = source.schema(capture)
            if (
                schema is None
                or schema.name not in _STRUCTURED
                or host.signature(channel) != source.signature(capture)
            ):
                raise ValueError(f"Conflicting MCAP topic: {channel.topic}")
            observed.add(channel.topic)
        sources = [host, *captures]
        expected = [sum(source.counts[i] for source in sources) for i in range(3)]
        with _partial(output).open("xb") as stream:
            writer = Writer(
                stream, compression=CompressionType.NONE, enable_data_crcs=True
            )
            writer.start(
                profile=host.reader.get_header().profile,
                library="isaaccapture.orbbec_wrist.merge",
            )
            for source in sources:
                schemas = {
                    key: writer.register_schema(value.name, value.encoding, value.data)
                    for key, value in source.summary.schemas.items()
                }
                for key, channel in source.summary.channels.items():
                    topic = (
                        _OBSERVED + channel.topic
                        if source is host and channel.topic in observed
                        else channel.topic
                    )
                    source.channels[key] = writer.register_channel(
                        topic,
                        channel.message_encoding,
                        schemas.get(channel.schema_id, 0),
                        channel.metadata,
                    )
                for metadata in source.reader.iter_metadata():
                    writer.add_metadata(metadata.name, metadata.metadata)
                for attachment in source.reader.iter_attachments():
                    writer.add_attachment(
                        attachment.create_time,
                        attachment.log_time,
                        attachment.name,
                        attachment.media_type,
                        attachment.data,
                    )
            writer.add_metadata(
                "orbbec_wrist.merge",
                {
                    "recording": str(recording.resolve()),
                    "media_fragments": json.dumps(
                        [str(path.resolve()) for path in fragments]
                    ),
                    "capture.channels": "complete plugin capture",
                    "session_observed.channels": "host session observations",
                },
            )

            # Separate file handles let the indexed readers advance independently.
            def messages(source):
                for _, channel, message in source.reader.iter_messages(
                    log_time_order=True
                ):
                    yield message.log_time, source.channels[channel.id], message

            count = 0
            for _, channel_id, message in heapq.merge(
                *(messages(source) for source in sources), key=lambda item: item[0]
            ):
                writer.add_message(
                    channel_id,
                    message.log_time,
                    message.data,
                    message.publish_time,
                    message.sequence,
                )
                count += 1
            if count != expected[0]:
                raise ValueError("Indexed MCAP merge did not preserve every message")
            writer.finish()
            stream.flush()
            os.fsync(stream.fileno())
        # Validate the completed file before exposing its final name.
        with _partial(output).open("rb") as stream:
            from mcap.reader import make_reader

            reader = make_reader(stream, validate_crcs=True)
            stats = reader.get_summary().statistics
            if [stats.message_count, stats.metadata_count, stats.attachment_count] != [
                expected[0],
                expected[1] + 1,
                expected[2],
            ] or sum(1 for _ in reader.iter_messages()) != expected[0]:
                raise ValueError("Merged MCAP failed its final inventory check")
        os.link(_partial(output), output)
        _partial(output).unlink()
