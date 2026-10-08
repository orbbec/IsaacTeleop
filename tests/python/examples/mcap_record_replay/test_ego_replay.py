# SPDX-FileCopyrightText: Copyright (c) 2026 ORBBEC INC.
# SPDX-License-Identifier: Apache-2.0

from types import SimpleNamespace

import pytest
from mcap.writer import CompressionType, IndexType, Writer

from isaaccapture_examples.mcap_record_replay import replay_ego

TOPICS = {
    "ego_metadata/ColorLeft",
    "ego_metadata/ColorRight",
    "ego_imu/Accel",
    "ego_imu/Gyro",
    "ego_audio/Audio",
    "ego_calibration/Calibration",
    "ego_device/DeviceState",
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
        for topic in topics | {"ego_audio/Audio_tracked"}:
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
    def __init__(self, *_):
        pass

    def get_data(self, session):
        if session.updates == 2:
            return None
        return SimpleNamespace(
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
            connection_state="Connected",
            reconnect_attempt=0,
        )

    def get_stream_data(self, session, _):
        return self.get_data(session)


def mock_trackers(monkeypatch):
    for name in (
        "EgoFrameMetadataTracker",
        "EgoImuTracker",
        "EgoAudioTracker",
        "EgoCalibrationTracker",
        "EgoDeviceStateTracker",
    ):
        monkeypatch.setattr(replay_ego, name, Tracker)
    session = Replay()
    monkeypatch.setattr(
        replay_ego, "ReplaySession", SimpleNamespace(run=lambda _: session)
    )
    monkeypatch.setattr(replay_ego, "McapReplayConfig", lambda *_: None)
    return session


@pytest.mark.parametrize("summary", [True, False])
def test_gap_does_not_end_replay_and_tracked_topics_do_not_extend_it(
    tmp_path, monkeypatch, capsys, summary
):
    recording = tmp_path / "gap.mcap"
    write_recording(recording, summary)
    session = mock_trackers(monkeypatch)
    counts = replay_ego.replay(recording, 0)
    assert session.updates == 3
    assert session.closed
    assert set(counts.values()) == {2}
    assert "ColorLeft: epoch=1 seq=3" in capsys.readouterr().out


def test_explicit_update_limit_includes_gaps(tmp_path, monkeypatch):
    recording = tmp_path / "gap.mcap"
    write_recording(recording, True)
    session = mock_trackers(monkeypatch)
    counts = replay_ego.replay(recording, 2)
    assert session.updates == 2
    assert set(counts.values()) == {1}


def test_recording_without_raw_topics_finishes_immediately(tmp_path, monkeypatch):
    recording = tmp_path / "other.mcap"
    write_recording(recording, True, {"head/head"})
    session = mock_trackers(monkeypatch)
    counts = replay_ego.replay(recording, 0)
    assert session.updates == 0
    assert set(counts.values()) == {0}
