# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from contextlib import contextmanager
from dataclasses import dataclass, field
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

from mcap.exceptions import McapError
from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from repo_paths import repo_root  # noqa: E402

PACKAGE = repo_root() / "src/python/isaaccapture/orbbec_wrist"
SPEC = importlib.util.spec_from_file_location(
    "orbbec_wrist_test_package", PACKAGE / "__init__.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
MERGE = sys.modules[SPEC.name + ".merge"]


def write_mcap(
    path,
    *,
    prefix=None,
    serial="S1",
    root="left",
    closed=True,
    media=True,
    empty_media=False,
    incompatible=False,
    observed=False,
    corrupt=False,
):
    with path.open("wb") as output:
        writer = Writer(output, compression=CompressionType.NONE, enable_data_crcs=True)
        writer.start()
        schema = writer.register_schema(
            "core.OrbbecWristFrameMetadataRecord",
            "flatbuffer",
            b"different" if incompatible else b"metadata-schema",
        )
        topic = (prefix or "wrist/left") + "/Color"
        channel = writer.register_channel(
            ("session_observed/" if observed else "") + topic,
            "flatbuffer",
            schema,
            {"source": root},
        )
        writer.add_message(channel, 20, b"metadata-" + root.encode(), 18, sequence=7)
        if prefix:
            writer.add_metadata(
                "wrist.capture",
                {
                    "fragment_version": "1",
                    "media_mode": "embedded",
                    "collection_prefix": prefix,
                    "plugin_root_id": root,
                    "serial_number": serial,
                },
            )
            writer.add_metadata(
                "wrist.calibration",
                {"json": "{}", "serial_number": serial, "collection_prefix": prefix},
            )
            schema = writer.register_schema(
                "core.OrbbecWristImuBatchRecord", "flatbuffer", b"imu-schema"
            )
            channel = writer.register_channel(
                prefix + "/Accel/Imu", "flatbuffer", schema
            )
            writer.add_message(channel, 21, b"imu-tail", 19, 8)
            if media:
                schema = writer.register_schema(
                    "core.OrbbecWristEncodedVideoFrameRecord",
                    "flatbuffer",
                    b"video-schema",
                )
                channel = writer.register_channel(
                    prefix + "/EncodedVideo/Color", "flatbuffer", schema
                )
                if not empty_media:
                    writer.add_message(
                        channel, 22, b"encoded-video-" + serial.encode(), 20, 9
                    )
            channel = writer.register_channel(prefix + "/Diagnostics", "json", 0)
            writer.add_message(
                channel,
                23,
                json.dumps({"state": 0 if closed else 2, "error": ""}).encode(),
                23,
            )
        else:
            schema = writer.register_schema("robot.pose", "jsonschema", b"{}")
            channel = writer.register_channel("robot/pose", "json", schema)
            writer.add_message(channel, 10, b'{"x":1}', 9, 6)
            writer.add_attachment(
                1, 2, "calibration", "application/json", b"calibration-bytes"
            )
        writer.finish()
    if corrupt:
        content = bytearray(path.read_bytes())
        offset = content.index(b"metadata-")
        content[offset] ^= 1
        path.write_bytes(content)
    return path


def read_messages(path):
    with path.open("rb") as stream:
        return [
            (
                channel.topic,
                message.log_time,
                message.publish_time,
                message.sequence,
                message.data,
            )
            for _, channel, message in make_reader(
                stream, validate_crcs=True
            ).iter_messages()
        ]


def test_merge_two_wrists_keeps_video_tail_timestamps_observations_and_other_devices(
    tmp_path,
):
    session = write_mcap(tmp_path / "session.mcap")
    left = write_mcap(tmp_path / "left.mcap", prefix="wrist/left")
    right = write_mcap(
        tmp_path / "right.mcap", prefix="wrist/right", serial="S2", root="right"
    )
    final = tmp_path / "final.mcap"
    before = {path: path.read_bytes() for path in (session, left, right)}
    MODULE.merge_recording(session, [left, right], final)
    messages = read_messages(final)
    expected = read_messages(session) + read_messages(left) + read_messages(right)
    expected = [
        (
            "session_observed/" + topic
            if topic == "wrist/left/Color" and data == b"metadata-left" and i < 2
            else topic,
            log,
            pub,
            seq,
            data,
        )
        for i, (topic, log, pub, seq, data) in enumerate(expected)
    ]
    assert sorted(messages) == sorted(expected)
    assert [item[1] for item in messages] == sorted(item[1] for item in messages)
    assert all(path.read_bytes() == content for path, content in before.items())
    assert not Path(str(final) + ".partial").exists()
    with final.open("rb") as stream:
        reader = make_reader(stream)
        assert [item.data for item in reader.iter_attachments()] == [
            b"calibration-bytes"
        ]
        captures = [
            item.metadata
            for item in reader.iter_metadata()
            if item.name == "wrist.capture"
        ]
        assert {item["serial_number"] for item in captures} == {"S1", "S2"}


@pytest.mark.parametrize(
    "failure",
    [
        "not_closed",
        "no_video",
        "empty_video",
        "crc",
        "partial",
        "truncated",
        "conflict",
        "reserved",
        "duplicate",
    ],
)
def test_merge_rejects_incomplete_or_conflicting_inputs(tmp_path, failure):
    session = write_mcap(
        tmp_path / "session.mcap",
        incompatible=failure == "conflict",
        observed=failure == "reserved",
    )
    fragment = write_mcap(
        tmp_path / "fragment.mcap",
        prefix="wrist/left",
        closed=failure != "not_closed",
        media=failure != "no_video",
        empty_media=failure == "empty_video",
        corrupt=failure == "crc",
    )
    if failure == "partial":
        Path(str(fragment) + ".partial").touch()
    if failure == "truncated":
        fragment.write_bytes(fragment.read_bytes()[:-16])
    fragments = [fragment]
    if failure == "duplicate":
        fragments.append(
            write_mcap(
                tmp_path / "second.mcap",
                prefix="wrist/right",
                serial="S1",
                root="right",
            )
        )
    final = tmp_path / "final.mcap"
    with pytest.raises((ValueError, McapError, EOFError)):
        MODULE.merge_recording(session, fragments, final)
    assert not final.exists()
    assert session.exists() and fragment.exists()


@pytest.mark.parametrize("which", ["final", "partial", "alias", "symlink"])
def test_merge_refuses_existing_and_alias_outputs(tmp_path, which):
    session = write_mcap(tmp_path / "session.mcap")
    fragment = write_mcap(tmp_path / "fragment.mcap", prefix="wrist/left")
    final = tmp_path / "final.mcap"
    if which == "final":
        final.write_bytes(b"keep")
    elif which == "partial":
        Path(str(final) + ".partial").write_bytes(b"keep")
    elif which == "alias":
        final = session
    else:
        final.symlink_to(tmp_path / "missing")
    original = session.read_bytes()
    with pytest.raises((ValueError, FileExistsError)):
        MODULE.merge_recording(session, [fragment], final)
    assert session.read_bytes() == original


def test_merge_write_failure_preserves_inputs_and_partial(tmp_path, monkeypatch):
    session = write_mcap(tmp_path / "session.mcap")
    fragment = write_mcap(tmp_path / "fragment.mcap", prefix="wrist/left")
    monkeypatch.setattr(MERGE.os, "fsync", Mock(side_effect=OSError("disk full")))
    final = tmp_path / "final.mcap"
    with pytest.raises(OSError, match="disk full"):
        MODULE.merge_recording(session, [fragment], final)
    assert not final.exists() and Path(str(final) + ".partial").exists()
    assert session.exists() and fragment.exists()


@dataclass
class Plugin:
    plugin_root_id: str
    plugin_args: list
    plugin_name: str = "orbbec_wrist_camera"
    search_paths: list = field(default_factory=list)
    enabled: bool = True
    required: bool = False


@dataclass
class Config:
    plugins: list
    mcap_config: object
    mode: str = "LIVE"


class McapConfig:
    def __init__(self, filename, trackers):
        self.filename, self.trackers = filename, trackers

    def get_tracker_names(self):
        return self.trackers


@pytest.fixture
def session_environment(tmp_path, monkeypatch):
    state = SimpleNamespace(fail=False, captured=None, calls=0, fragment_fault=None)

    @contextmanager
    def session(config):
        state.calls += 1
        state.captured = config
        yield state
        if state.fail:
            raise RuntimeError("shutdown failed")
        write_mcap(Path(config.mcap_config.filename))
        for plugin in config.plugins:
            if not plugin.enabled or plugin.plugin_name != "orbbec_wrist_camera":
                continue
            args = dict(
                arg[2:].split("=", 1) for arg in plugin.plugin_args if "=" in arg
            )
            write_mcap(
                Path(args["mcap-media-spool"]),
                prefix=args["collection-prefix"],
                serial=args.get("serial-number", "S1"),
                root=plugin.plugin_root_id,
            )
            fragment = Path(args["mcap-media-spool"])
            if state.fragment_fault == "missing":
                fragment.unlink()
            elif state.fragment_fault == "partial":
                Path(str(fragment) + ".partial").touch()
            elif state.fragment_fault == "identity":
                write_mcap(
                    fragment,
                    prefix=args["collection-prefix"],
                    serial="WRONG",
                    root="another_root",
                )

    manager = Mock()
    manager.return_value.get_plugin_info.return_value = SimpleNamespace(
        args=[], working_dir=str(tmp_path)
    )
    monkeypatch.setitem(
        sys.modules,
        "isaaccapture.deviceio_session",
        SimpleNamespace(McapRecordingConfig=McapConfig),
    )
    monkeypatch.setitem(
        sys.modules,
        "isaaccapture.plugin_manager",
        SimpleNamespace(PluginManager=manager),
    )
    monkeypatch.setitem(
        sys.modules,
        "isaaccapture.teleop_session_manager",
        SimpleNamespace(
            TeleopSession=session, SessionMode=SimpleNamespace(LIVE="LIVE")
        ),
    )
    return state


def configuration(tmp_path):
    plugins = [
        Plugin(root, [f"--collection-prefix=wrist/{root}", f"--serial-number=S{i}"])
        for i, root in enumerate(("left", "right"), 1)
    ]
    config = Config(
        plugins, McapConfig(str(tmp_path / "final.mcap"), [("tracker", "name")])
    )
    fragments = {root: tmp_path / (root + ".mcap") for root in ("left", "right")}
    return config, fragments


def test_session_helper_configures_both_plugins_and_finalizes_without_mutating_caller(
    tmp_path, session_environment
):
    config, fragments = configuration(tmp_path)
    with MODULE.embedded_recording(config, media_fragments=fragments) as session:
        assert session is session_environment
        assert not (tmp_path / "final.mcap").exists()
    assert (tmp_path / "final.mcap").exists()
    assert all(plugin.required for plugin in session_environment.captured.plugins)
    assert all(not plugin.required for plugin in config.plugins)
    assert not any(
        "spool" in arg for plugin in config.plugins for arg in plugin.plugin_args
    )
    assert config.mcap_config.filename == str(tmp_path / "final.mcap")
    assert session_environment.captured.mcap_config.get_tracker_names() == [
        ("tracker", "name")
    ]


@pytest.mark.parametrize("failure", ["shutdown", "body"])
def test_session_helper_never_merges_after_failure(
    tmp_path, session_environment, failure
):
    config, fragments = configuration(tmp_path)
    session_environment.fail = failure == "shutdown"
    with pytest.raises(RuntimeError):
        with MODULE.embedded_recording(config, media_fragments=fragments):
            if failure == "body":
                raise RuntimeError("body failed")
    assert not (tmp_path / "final.mcap").exists()


@pytest.mark.parametrize(
    "invalid",
    [
        "missing_map",
        "same_prefix",
        "same_serial",
        "no_serial",
        "same_path",
        "wrong_spool",
        "sidecar",
        "local_mcap",
        "wrong_root",
    ],
)
def test_session_preflight_fails_before_starting_plugins(
    tmp_path, session_environment, invalid
):
    config, fragments = configuration(tmp_path)
    args = config.plugins[1].plugin_args
    if invalid == "missing_map":
        del fragments["right"]
    elif invalid == "same_prefix":
        args[0] = "--collection-prefix=wrist/left"
    elif invalid == "same_serial":
        args[1] = "--serial-number=S1"
    elif invalid == "no_serial":
        args.pop()
    elif invalid == "same_path":
        fragments["right"] = fragments["left"]
    elif invalid == "wrong_spool":
        args.append("--mcap-media-spool=wrong.mcap")
    elif invalid == "sidecar":
        args.append("--media-mode=sidecar")
    elif invalid == "local_mcap":
        args.append("--mcap-filename=local.mcap")
    else:
        args.append("--plugin-root-id=wrong")
    with pytest.raises(ValueError):
        with MODULE.embedded_recording(config, media_fragments=fragments):
            pytest.fail("must fail before yielding")
    assert session_environment.calls == 0


def test_merge_output_race_never_replaces_another_file(tmp_path, monkeypatch):
    session = write_mcap(tmp_path / "session.mcap")
    fragment = write_mcap(tmp_path / "fragment.mcap", prefix="wrist/left")
    final = tmp_path / "final.mcap"
    link = MERGE.os.link

    def race(source, destination):
        final.write_bytes(b"another process")
        return link(source, destination)

    monkeypatch.setattr(MERGE.os, "link", race)
    with pytest.raises(FileExistsError):
        MODULE.merge_recording(session, [fragment], final)
    assert final.read_bytes() == b"another process"
    assert Path(str(final) + ".partial").exists()


def test_session_accepts_one_automatic_device_and_matching_relative_spool(
    tmp_path, session_environment
):
    config, fragments = configuration(tmp_path)
    config.plugins = config.plugins[:1]
    config.plugins[0].plugin_args = [
        "--collection-prefix=wrist/left",
        "--mcap-media-spool=left.mcap",
    ]
    with MODULE.embedded_recording(config, media_fragments={"left": fragments["left"]}):
        pass
    assert (tmp_path / "final.mcap").exists()


@pytest.mark.parametrize("fault", ["missing", "partial", "identity"])
def test_session_does_not_finalize_missing_uncommitted_or_wrong_device_fragments(
    tmp_path, session_environment, fault
):
    config, fragments = configuration(tmp_path)
    session_environment.fragment_fault = fault
    with pytest.raises((ValueError, FileNotFoundError)):
        with MODULE.embedded_recording(config, media_fragments=fragments):
            pass
    assert not (tmp_path / "final.mcap").exists()
    assert (tmp_path / "final.session.mcap").exists()


def test_session_preserves_other_and_disabled_plugins(tmp_path, session_environment):
    config, fragments = configuration(tmp_path)
    other = Plugin("robot", [], plugin_name="robot_plugin")
    disabled = Plugin("disabled", [], enabled=False)
    config.plugins.extend([other, disabled])
    with MODULE.embedded_recording(config, media_fragments=fragments):
        pass
    assert session_environment.captured.plugins[2] is other
    assert session_environment.captured.plugins[3] is disabled
