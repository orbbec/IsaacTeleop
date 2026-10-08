# SPDX-FileCopyrightText: Copyright (c) 2026 ORBBEC INC. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path
from types import SimpleNamespace
import subprocess

import pytest

from isaaccapture import plugin_manager, teleop_session_manager
from isaaccapture.deviceio_session import McapRecordingConfig
from isaaccapture.ego import embedded_recording
from isaaccapture.ego import recording
from isaaccapture.teleop_session_manager import (
    PluginConfig,
    SessionMode,
    TeleopSessionConfig,
)


@pytest.fixture
def setup(tmp_path, monkeypatch):
    fragment = tmp_path / "capture.mcap"
    final = tmp_path / "final.mcap"
    executable = tmp_path / "ego_mcap_merge"
    executable.write_text("merge fixture")
    executable.chmod(0o755)
    plugin = PluginConfig(
        "ego_camera",
        "/camera/ego",
        [tmp_path],
        plugin_args=[
            "--collection-prefix=ego",
            "--mcap-media=embedded",
            f"--mcap-media-spool={fragment}",
        ],
    )
    config = TeleopSessionConfig(
        "test", object(), plugins=[plugin], mcap_config=McapRecordingConfig(str(final))
    )
    events = []
    state = SimpleNamespace(
        config=config,
        fragment=fragment,
        final=final,
        executable=executable,
        events=events,
        finish="formal",
        cleanup_error=False,
        merger_error=False,
        session_config=None,
        command=None,
    )
    info = SimpleNamespace(working_dir=str(tmp_path), args=[])
    monkeypatch.setattr(
        plugin_manager,
        "PluginManager",
        lambda paths: SimpleNamespace(get_plugin_info=lambda name: info),
    )

    class Session:
        def __init__(self, session_config):
            state.session_config = session_config

        def __enter__(self):
            events.append("enter")
            Path(state.session_config.mcap_config.filename).write_bytes(
                b"session fixture"
            )
            return self

        def __exit__(self, *exception):
            events.append("exit")
            if state.finish == "formal":
                fragment.write_bytes(b"capture fixture")
            elif state.finish == "partial":
                Path(str(fragment) + ".partial").write_bytes(b"incomplete fixture")
            if state.cleanup_error:
                raise OSError("cleanup failed")
            return False

    def merge(command, *, check):
        assert events[-1] == "exit"
        assert check
        events.append("merge")
        state.command = command
        if state.merger_error:
            Path(str(final) + ".partial").write_bytes(b"partial merge")
            raise subprocess.CalledProcessError(1, command)
        final.write_bytes(b"merged fixture")

    monkeypatch.setattr(teleop_session_manager, "TeleopSession", Session)
    monkeypatch.setattr(recording.subprocess, "run", merge)
    return state


def test_merge_follows_complete_session_cleanup_without_mutating_config(setup):
    original = setup.config.mcap_config
    with embedded_recording(setup.config, media_fragment=setup.fragment) as session:
        assert session is not None
        setup.events.append("body")
    assert setup.events == ["enter", "body", "exit", "merge"]
    assert setup.config.mcap_config is original
    assert setup.config.mcap_config.filename == str(setup.final)
    assert not setup.config.plugins[0].required
    assert setup.session_config.plugins[0].required
    assert setup.command == [
        str(setup.executable),
        "--recording",
        str(setup.final.with_name("final.session.mcap")),
        "--media",
        str(setup.fragment),
        "--output",
        str(setup.final),
    ]
    assert setup.fragment.exists()
    assert setup.final.with_name("final.session.mcap").exists()


def test_body_error_propagates_and_keeps_inputs_without_merging(setup):
    with pytest.raises(RuntimeError, match="body failed"):
        with embedded_recording(setup.config, media_fragment=setup.fragment):
            raise RuntimeError("body failed")
    assert setup.events == ["enter", "exit"]
    assert not setup.final.exists()
    assert setup.fragment.exists()


def test_cleanup_error_is_not_reported_as_complete_recording(setup):
    setup.cleanup_error = True
    with pytest.raises(OSError, match="cleanup failed"):
        with embedded_recording(setup.config, media_fragment=setup.fragment):
            pass
    assert setup.events == ["enter", "exit"]
    assert not setup.final.exists()


@pytest.mark.parametrize("finish", ["partial", "missing"])
def test_stop_without_committed_fragment_does_not_merge(setup, finish):
    setup.finish = finish
    with pytest.raises(RuntimeError, match="did not commit"):
        with embedded_recording(setup.config, media_fragment=setup.fragment):
            pass
    assert setup.events == ["enter", "exit"]
    assert not setup.final.exists()


def test_merge_failure_propagates_and_retains_evidence(setup):
    setup.merger_error = True
    with pytest.raises(subprocess.CalledProcessError):
        with embedded_recording(setup.config, media_fragment=setup.fragment):
            pass
    assert setup.fragment.exists()
    assert setup.final.with_name("final.session.mcap").exists()
    assert Path(str(setup.final) + ".partial").exists()
    assert not setup.final.exists()


@pytest.mark.parametrize(
    "invalid",
    [
        "replay",
        "no_recording",
        "duplicate",
        "spool",
        "mode",
        "missing_merger",
        "existing_output",
    ],
)
def test_invalid_recording_fails_before_capture(setup, invalid):
    if invalid == "replay":
        setup.config.mode = SessionMode.REPLAY
    elif invalid == "no_recording":
        setup.config.mcap_config = None
    elif invalid == "duplicate":
        setup.config.plugins.append(setup.config.plugins[0])
    elif invalid == "spool":
        setup.config.plugins[0].plugin_args[-1] = "--mcap-media-spool=other.mcap"
    elif invalid == "mode":
        setup.config.plugins[0].plugin_args[1] = "--mcap-media=metadata-only"
    elif invalid == "missing_merger":
        setup.executable.unlink()
    elif invalid == "existing_output":
        setup.final.write_bytes(b"keep")
    with pytest.raises((ValueError, FileNotFoundError, FileExistsError)):
        with embedded_recording(setup.config, media_fragment=setup.fragment):
            pytest.fail("Capture must not start")
    assert not setup.events
    if invalid == "existing_output":
        assert setup.final.read_bytes() == b"keep"


def test_caught_interrupt_is_a_normal_stop(setup):
    with embedded_recording(setup.config, media_fragment=setup.fragment):
        try:
            raise KeyboardInterrupt
        except KeyboardInterrupt:
            pass
    assert setup.events == ["enter", "exit", "merge"]
