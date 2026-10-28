# SPDX-FileCopyrightText: Copyright (c) 2026 ORBBEC INC.
# SPDX-License-Identifier: Apache-2.0

from types import SimpleNamespace

import pytest
from mcap.writer import CompressionType, IndexType, Writer

from isaaccapture_examples.orbbec_ego import replay
from isaaccapture.schema import (
    OrbbecEgoRecoveryEvent,
    OrbbecEgoRecoveryStage,
    OrbbecEgoDiagnosticEvent,
)

TOPICS = {
    "orbbec_ego_metadata_colorleft/FrameMetadata",
    "orbbec_ego_metadata_colorright/FrameMetadata",
    "orbbec_ego_imu_accel/Imu",
    "orbbec_ego_imu_gyro/Imu",
    "orbbec_ego_audio/Audio",
    "orbbec_ego_calibration/Calibration",
    "orbbec_ego_device/DeviceState",
}


def write_recording(path, summary, topics=TOPICS):
    with path.open("wb") as stream:
        writer = Writer(
            stream,
            compression=CompressionType.NONE,
            use_chunking=False,
            index_types=IndexType.NONE,
            repeat_schemas=summary,
            repeat_channels=summary,
            use_statistics=summary,
            use_summary_offsets=summary,
        )
        writer.start()
        for topic in topics | {"orbbec_ego_audio/Audio_tracked"}:
            channel = writer.register_channel(topic, "test", 0)
            for sequence in range(1, 4 if topic in topics else 10):
                writer.add_message(channel, sequence, b"sample", sequence)
        writer.finish()


class Replay:
    def __init__(self):
        self.updates = 0
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.closed = True

    def update(self):
        self.updates += 1


class Tracker:
    def __init__(self, prefix):
        self.prefix = prefix

    def get_data(self, session):
        if session.updates == 2:
            return None
        data = SimpleNamespace(
            capture_epoch=1,
            sequence_number=session.updates,
            width=1600,
            height=1300,
            fps=30,
            pixel_format="H264",
            encoded_bytes=100,
            sample_rate_hz=48000,
            samples=[1],
            sample_count=480,
            byte_count=960,
            device_uid="ego",
            event_type=OrbbecEgoDiagnosticEvent.RecoveryEvent,
            diagnostic_version=2,
            observed_at_ns=100,
            event=OrbbecEgoRecoveryEvent(
                OrbbecEgoRecoveryStage.Recovered, "recovered", 1
            ),
        )

        name = self.prefix.rsplit("/", 1)[-1]
        if name in replay.VIDEO_STREAM_NAMES:
            data.stream = getattr(replay.OrbbecEgoCameraStream, name)
        elif name in ("Accel", "Gyro"):
            data.sensor = getattr(replay.OrbbecEgoImuSensor, name)
        return data


def mock_trackers(monkeypatch):
    for name in (
        "OrbbecEgoFrameMetadataTracker",
        "OrbbecEgoImuTracker",
        "OrbbecEgoAudioTracker",
        "OrbbecEgoCalibrationTracker",
        "OrbbecEgoDeviceStateTracker",
    ):
        monkeypatch.setattr(replay, name, Tracker)
    session = Replay()

    def run(config):
        session.trackers = config.trackers
        return session

    monkeypatch.setattr(replay, "ReplaySession", SimpleNamespace(run=run))
    monkeypatch.setattr(
        replay,
        "McapReplayConfig",
        lambda path, trackers: SimpleNamespace(path=path, trackers=trackers),
    )
    return session


@pytest.mark.parametrize("summary", [True, False])
def test_gap_does_not_end_replay_and_tracked_topics_do_not_extend_it(
    tmp_path, monkeypatch, capsys, summary
):
    recording = tmp_path / "gap.mcap"
    write_recording(recording, summary)
    session = mock_trackers(monkeypatch)
    counts = replay.replay(recording, 0)
    assert session.updates == 3
    assert session.closed
    assert set(counts.values()) == {2}
    assert "ColorLeft: epoch=1 seq=3" in capsys.readouterr().out


def test_explicit_update_limit_includes_gaps(tmp_path, monkeypatch):
    recording = tmp_path / "gap.mcap"
    write_recording(recording, True)
    session = mock_trackers(monkeypatch)
    counts = replay.replay(recording, 2)
    assert session.updates == 2
    assert set(counts.values()) == {1}


def test_recording_without_raw_topics_finishes_immediately(tmp_path, monkeypatch):
    recording = tmp_path / "other.mcap"
    write_recording(recording, True, {"head/head"})
    session = mock_trackers(monkeypatch)
    counts = replay.replay(recording, 0)
    assert session.updates == 0
    assert set(counts.values()) == {0}


@pytest.mark.parametrize("summary", [True, False])
def test_quad_video_replay_without_optional_sensors(tmp_path, monkeypatch, summary):
    recording = tmp_path / "quad.mcap"
    topics = {
        f"orbbec_ego_metadata_{name.lower()}/FrameMetadata"
        for name in replay.VIDEO_STREAM_NAMES
    } | {
        "orbbec_ego_calibration/Calibration",
        "orbbec_ego_device/DeviceState",
    }
    write_recording(recording, summary, topics)
    session = mock_trackers(monkeypatch)
    counts = replay.replay(recording, 0)
    assert session.updates == 3
    assert set(counts) == set(replay.VIDEO_STREAM_NAMES) | {
        "Calibration",
        "DeviceState",
    }
    assert set(counts.values()) == {2}
    registered = {name: tracker.prefix for tracker, name in session.trackers}
    assert {
        name: prefix
        for name, prefix in registered.items()
        if name.startswith("orbbec_ego_metadata_")
    } == {
        f"orbbec_ego_metadata_{name.lower()}": f"ego/{name}"
        for name in replay.VIDEO_STREAM_NAMES
    }


def test_empty_optional_channels_are_not_required(tmp_path, monkeypatch):
    recording = tmp_path / "empty_aux.mcap"
    with recording.open("wb") as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start()
        for topic in TOPICS:
            channel = writer.register_channel(topic, "test", 0)
            if not topic.startswith(("orbbec_ego_imu_", "orbbec_ego_audio/")):
                writer.add_message(channel, 1, b"test", 1)
        writer.finish()
    mock_trackers(monkeypatch)
    counts = replay.replay(recording, 0)
    assert not {"Accel", "Gyro", "Audio"}.intersection(counts)
    assert set(counts.values()) == {1}


def test_video_topic_payload_mismatch_is_rejected(tmp_path, monkeypatch):
    recording = tmp_path / "wrong_stream.mcap"
    write_recording(
        recording, True, {"orbbec_ego_metadata_colorsideright/FrameMetadata"}
    )
    mock_trackers(monkeypatch)

    original_get_data = Tracker.get_data

    def wrong_stream(self, session):
        data = original_get_data(self, session)
        data.stream = replay.OrbbecEgoCameraStream.ColorLeft
        return data

    monkeypatch.setattr(Tracker, "get_data", wrong_stream)
    with pytest.raises(ValueError, match=r"ColorSideRight.*does not match"):
        replay.replay(recording, 0)


def test_unknown_video_topic_is_rejected(tmp_path):
    recording = tmp_path / "unknown.mcap"
    write_recording(recording, True, {"orbbec_ego_metadata_colorunknown/FrameMetadata"})
    with pytest.raises(ValueError, match="Unknown EGO video"):
        replay.replay(recording, 0)


@pytest.mark.parametrize("enabled", [True, False])
def test_recorded_policy_controls_diagnostic_tracker_registration(
    tmp_path, monkeypatch, enabled
):
    path = tmp_path / "policy.mcap"
    with path.open("wb") as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start()
        writer.add_metadata(
            "orbbec_ego.recording",
            {
                "record_device_diagnostics": "on" if enabled else "off",
                "diagnostic_version": "2",
            },
        )
        channel = writer.register_channel(
            "orbbec_ego_metadata_colorleft/FrameMetadata", "test", 0
        )
        writer.add_message(channel, 1, b"test", 1)
        writer.finish()
    mock_trackers(monkeypatch)
    counts = replay.replay(path, 0)
    assert ("DeviceState" in counts) == enabled


def test_off_rejects_diagnostic_channel_under_a_different_topic(tmp_path):
    path = tmp_path / "conflict.mcap"
    with path.open("wb") as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start()
        writer.add_metadata(
            "orbbec_ego.recording",
            {"record_device_diagnostics": "off", "diagnostic_version": "2"},
        )
        schema = writer.register_schema(
            "core.OrbbecEgoDeviceStateRecord", "flatbuffer", b"fixture"
        )
        writer.register_channel("custom/topic", "flatbuffer", schema)
        writer.finish()
    with pytest.raises(ValueError, match="channel exists"):
        replay.replay(path, 0)


@pytest.mark.parametrize("version", ["0", "1", "99"])
def test_unsupported_diagnostic_policy_is_rejected(tmp_path, version):
    path = tmp_path / "unsupported.mcap"
    with path.open("wb") as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start()
        writer.add_metadata(
            "orbbec_ego.recording",
            {"record_device_diagnostics": "on", "diagnostic_version": version},
        )
        writer.finish()
    with pytest.raises(ValueError, match="Unsupported EGO"):
        replay.replay(path, 0)
