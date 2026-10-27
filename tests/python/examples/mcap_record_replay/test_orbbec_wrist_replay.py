# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import sys
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
from mcap.writer import CompressionType, IndexType, Writer

from isaaccapture_examples.mcap_record_replay import replay_orbbec_wrist as example


def write_recording(path, prefix="wrist_camera", *, summary=True, imu=True):
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
        sizes = {"Color": 3, "Color_tracked": 10, "EncodedVideo/Color": 10}
        if imu:
            sizes.update({"Accel/Imu": 4, "Gyro/Imu": 2})
        for suffix, size in sizes.items():
            channel = writer.register_channel(f"{prefix}/{suffix}", "test", 0)
            for sequence in range(1, size + 1):
                writer.add_message(channel, sequence, b"sample", sequence)
        writer.finish()


def mock_replay(monkeypatch, prefix="wrist_camera", *, imu=True):
    session = SimpleNamespace(updates=0, closed=False)

    def update():
        session.updates += 1

    session.update = update

    class Tracker:
        def __init__(self, collection):
            self.name = collection.rsplit("/", 1)[-1]

        def get_data(self, session):
            last = {"wrist_metadata": 3, "Accel": 4, "Gyro": 2}[self.name]
            if session.updates == 2 or session.updates > last:
                return None
            return SimpleNamespace(
                capture_epoch=1,
                sequence_number=session.updates,
                width=1280,
                height=720,
                fps=30,
                pixel_format="H264",
                encoded_bytes=100,
                sample_rate_hz=200,
                samples=[1, 2],
            )

    def config(path, trackers):
        expected = [prefix]
        if imu:
            expected += [f"{prefix}/Accel", f"{prefix}/Gyro"]
        assert [topic for _, topic in trackers] == expected
        assert len({id(tracker) for tracker, _ in trackers}) == len(expected)
        return path

    @contextmanager
    def run(_):
        try:
            yield session
        finally:
            session.closed = True

    monkeypatch.setattr(example, "OrbbecWristFrameMetadataTracker", Tracker)
    monkeypatch.setattr(example, "OrbbecWristImuTracker", Tracker)
    monkeypatch.setattr(example, "McapReplayConfig", config)
    monkeypatch.setattr(example, "ReplaySession", SimpleNamespace(run=run))
    return session


@pytest.mark.parametrize("summary", [True, False])
@pytest.mark.parametrize("prefix", ["wrist_camera", "orbbec_wrist/camera_1"])
def test_independent_streams_reach_eof_despite_gaps(
    tmp_path, monkeypatch, capsys, summary, prefix
):
    path = tmp_path / "capture.mcap"
    write_recording(path, prefix, summary=summary)
    session = mock_replay(monkeypatch, prefix)

    counts = example.replay(path, topic_prefix=prefix)

    assert counts == {"Color": 2, "Accel": 3, "Gyro": 1}
    assert session.updates == 4
    assert session.closed
    assert "Color: epoch=1 seq=3" in capsys.readouterr().out


def test_update_limit_counts_payload_less_records(tmp_path, monkeypatch):
    path = tmp_path / "capture.mcap"
    write_recording(path)
    session = mock_replay(monkeypatch)
    assert example.replay(path, 2) == {"Color": 1, "Accel": 1, "Gyro": 1}
    assert session.updates == 2
    assert session.closed


def test_color_only_cli(tmp_path, monkeypatch, capsys):
    path = tmp_path / "capture.mcap"
    write_recording(path, imu=False)
    session = mock_replay(monkeypatch, imu=False)
    monkeypatch.setattr(sys, "argv", ["replay", str(path), "--no-imu"])

    assert example.main() == 0
    assert session.updates == 3
    assert "Replay counts: Color=2" in capsys.readouterr().out


def test_wrong_prefix_reports_missing_payloads(tmp_path, monkeypatch):
    path = tmp_path / "other.mcap"
    write_recording(path, "other_camera")
    session = mock_replay(monkeypatch)
    monkeypatch.setattr(sys, "argv", ["replay", str(path)])

    assert example.main() == 1
    assert session.updates == 0


@pytest.mark.parametrize("args", [["--max-updates", "-1"], ["--topic-prefix", "/"]])
def test_invalid_cli_options(tmp_path, monkeypatch, args):
    path = tmp_path / "capture.mcap"
    path.touch()
    monkeypatch.setattr(sys, "argv", ["replay", str(path), *args])
    with pytest.raises(SystemExit) as exc:
        example.main()
    assert exc.value.code == 2
