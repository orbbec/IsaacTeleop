# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Wrist producer cleanup, camera identity and encoded recovery contracts."""

import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
import yaml

from repo_paths import repo_root
from sources import build_local_camera
from sources.orbbec_wrist import OrbbecWristSource


@pytest.fixture
def preview(monkeypatch):
    cp = MagicMock()
    cp.uint8 = np.uint8
    cp.empty.side_effect = lambda *args, **kwargs: np.empty((1, 1, 4), np.uint8)
    cp.cuda.runtime.getDevice.return_value = 0
    capture = MagicMock()
    capture.return_value.serial_number = "orbbec_wrist-123"
    monkeypatch.setitem(sys.modules, "cupy", cp)
    monkeypatch.setitem(
        sys.modules, "orbbec_wrist_preview", SimpleNamespace(Capture=capture)
    )
    source = OrbbecWristSource("orbbec_wrist-test", renderer_cuda_device_id=2)
    yield source, cp, capture
    source.stop()


def run_frames(preview, monkeypatch, frames):
    source, _, capture = preview
    decoder = MagicMock()
    decoder.decode.return_value = True
    monkeypatch.setattr("sources.orbbec_wrist.NvVideoDecoder", decoder)
    # The factory is supplied explicitly to exercise the real recovery gate.
    from sources.orbbec_wrist import EncodedVideoDecoder

    monkeypatch.setattr(
        "sources.orbbec_wrist.EncodedVideoDecoder",
        lambda *args, **kwargs: EncodedVideoDecoder(
            *args, **kwargs, decoder_factory=lambda *a, **k: decoder
        ),
    )
    pending = iter(frames)

    def next_frame(timeout):
        try:
            frame = next(pending)
        except StopIteration:
            source._stop.set()
            return None
        if isinstance(frame, Exception):
            raise frame
        return frame

    capture.return_value.next_frame.side_effect = next_frame
    source.start()
    source._thread.join(timeout=3)
    assert not source._thread.is_alive()
    return decoder


def packet(sequence, data):
    return {"data": data, "sequence": sequence, "timestamp_ns": sequence + 100}


PARAMS = b"\x00\x00\x01\x67\x80\x00\x00\x01\x68\x80"
KEY = b"\x00\x00\x01\x65\x80"
PREDICTED = b"\x00\x00\x01\x61\x80"


def test_example_is_mono_and_does_not_open_hardware():
    cfg = yaml.safe_load(
        (repo_root() / "examples/camera_viz/configs/orbbec_wrist.yaml").read_text()
    )
    (source,) = build_local_camera(cfg["cameras"][0])
    assert isinstance(source, OrbbecWristSource)
    assert (source.spec.width, source.spec.height) == (1600, 1200)
    assert source.latest() is None and source._capture is None


@pytest.mark.parametrize(
    "options,match",
    [
        ({"stereo": True}, "mono"),
        ({"width": 640}, "1600x1200"),
        ({"height": 1300}, "1600x1200"),
        ({"fps": 0}, "positive"),
        ({"format": "rgb"}, "format"),
    ],
)
def test_invalid_profiles_fail_before_gpu_or_sdk_access(options, match):
    with pytest.raises(ValueError, match=match):
        build_local_camera({"name": "orbbec_wrist", "type": "orbbec_wrist", **options})


def test_gap_waits_for_parameters_and_keyframe(preview, monkeypatch):
    source, cp, capture = preview
    decoder = run_frames(
        preview,
        monkeypatch,
        [
            packet(0, PARAMS + KEY),
            None,
            packet(2, PREDICTED),
            packet(3, PARAMS),
            packet(4, KEY),
        ],
    )
    assert source._frames == 2
    decoder.reset.assert_called_once()
    assert [c.args[0] for c in decoder.decode.call_args_list] == [PARAMS + KEY] * 2
    frame = source.latest()
    assert frame.timestamp_ns == 104 and frame.image_right is None
    assert source.latest() is None
    assert all(c.args == (2,) for c in cp.cuda.Device.call_args_list)
    capture.return_value.close.assert_called_once()


def test_reconnect_pins_serial_and_resets_decoder(preview, monkeypatch):
    source, _, capture = preview
    decoder = run_frames(
        preview,
        monkeypatch,
        [
            packet(0, PARAMS),
            RuntimeError("read failed"),
            packet(1, KEY),
            packet(2, PARAMS + KEY),
        ],
    )
    assert [c.args[0] for c in capture.call_args_list] == ["", "orbbec_wrist-123"]
    assert capture.return_value.close.call_count == 2
    decoder.reset.assert_called_once()
    assert source._frames == 1
    assert source.latest().timestamp_ns == 102


def test_decode_error_reaches_consumer_and_closes_sdk(preview, monkeypatch):
    source, _, capture = preview
    failure = RuntimeError("GPU decode failed")
    decoder = MagicMock()
    decoder.decode.side_effect = failure
    monkeypatch.setattr(
        "sources.orbbec_wrist.EncodedVideoDecoder", lambda *a, **k: decoder
    )
    capture.return_value.next_frame.return_value = packet(0, PARAMS + KEY)
    source.start()
    source._thread.join(timeout=2)
    assert not source._thread.is_alive()
    with pytest.raises(RuntimeError, match="GPU decode failed") as error:
        source.latest()
    assert error.value.__cause__ is failure
    capture.return_value.close.assert_called_once()


@pytest.mark.parametrize("requested,renderer", [(1, 2), (2, 0), (-1, 0), (0, -1)])
def test_gpu_conflicts_fail_before_opening_capture(preview, requested, renderer):
    source, _, capture = preview
    source._requested_gpu_id = requested
    source._renderer_cuda_device_id = renderer
    with pytest.raises(ValueError, match="CUDA"):
        source.start()
    capture.assert_not_called()


def test_stop_retains_capture_while_producer_is_live():
    source = OrbbecWristSource("orbbec_wrist")
    source._thread = MagicMock()
    source._thread.is_alive.return_value = True
    capture = object()
    source._capture = capture
    source.stop()
    assert source._capture is capture
    source._thread.join.assert_called_once_with(timeout=5.0)


@pytest.mark.parametrize("dependency", ["cupy", "orbbec_wrist_preview"])
def test_missing_capture_dependency_fails_before_producer_start(
    preview, monkeypatch, dependency
):
    source, _, capture = preview
    monkeypatch.setitem(sys.modules, dependency, None)
    with pytest.raises(RuntimeError, match="setup --with-orbbec-wrist"):
        source.start()
    assert source._thread is None
    assert source._capture is None
    capture.assert_not_called()


def test_missing_native_decoder_reports_build_requirement(monkeypatch):
    from sources._nv_decode import NvVideoDecoder

    monkeypatch.setitem(sys.modules, "codec", None)
    decoder = NvVideoDecoder(1600, 1200)
    with pytest.raises(RuntimeError, match=r"codec/build\.sh"):
        decoder.decode(b"", None)
    assert decoder._decoder is None


def test_decoder_initialization_error_reaches_consumer_before_opening_sdk(
    preview, monkeypatch
):
    source, _, capture = preview
    failure = RuntimeError("decoder initialization failed")

    def fail_initialization(*args, **kwargs):
        raise failure

    monkeypatch.setattr("sources.orbbec_wrist.EncodedVideoDecoder", fail_initialization)
    source.start()
    source._thread.join(timeout=2)
    assert not source._thread.is_alive()
    with pytest.raises(RuntimeError, match="decoder initialization failed") as error:
        source.latest()
    assert error.value.__cause__ is failure
    capture.assert_not_called()
    assert source._capture is None
