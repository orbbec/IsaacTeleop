# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import json
from pathlib import Path
import shutil
import shlex
import subprocess
import sys
from unittest.mock import patch

import flatbuffers
from mcap.writer import CompressionType, Writer
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from repo_paths import repo_root  # noqa: E402

PLUGIN = repo_root() / "src/plugins/orbbec_wrist"
SPEC = importlib.util.spec_from_file_location(
    "wrist_verify", PLUGIN / "tools/verify_recording.py"
)
VERIFIER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VERIFIER)


def payload(embedded=False, encoded_data=b"test"):
    builder = flatbuffers.Builder(256)
    encoded = builder.CreateByteVector(encoded_data) if embedded else None
    builder.StartObject(12)
    builder.PrependUint64Slot(1, 1, 0)
    builder.PrependUint32Slot(2, 1600, 0)
    builder.PrependUint32Slot(3, 1200, 0)
    builder.PrependUint32Slot(4, 30, 0)
    builder.PrependInt8Slot(5, 1, 0)
    if embedded:
        builder.PrependUOffsetTRelativeSlot(6, encoded, 0)
    else:
        builder.PrependUint64Slot(6, len(encoded_data), 0)
        builder.PrependUint64Slot(8, 1000000, 0)
    builder.PrependUint32Slot(7, 1, 0)
    table = builder.EndObject()
    builder.StartObject(2)
    builder.PrependUOffsetTRelativeSlot(0, table, 0)
    builder.Finish(builder.EndObject())
    return bytes(builder.Output())


@pytest.fixture
def recording(tmp_path):
    def create(mode="sidecar", closed=True, encoded_data=b"test"):
        (tmp_path / "raw").mkdir(exist_ok=True)
        (tmp_path / "logs").mkdir(exist_ok=True)
        (tmp_path / "raw/Color.h264").write_bytes(encoded_data)
        (tmp_path / "logs/capture.log").write_text("capture finalized\n")
        (tmp_path / "capabilities.txt").write_text("Sensor Color\n")
        calibration = {
            "serial_number": "test-wrist",
            "color": None,
            "errors": {"color": "unavailable"},
        }
        (tmp_path / "calibration.json").write_text(json.dumps(calibration))
        manifest = {
            "version": 1,
            "media_mode": mode,
            "streams": {
                "Color": {"format": "h264", "width": 1600, "height": 1200, "fps": 30}
            },
            "imu": {"enabled": False, "sample_rate_hz": 400},
        }
        (tmp_path / "requested_profile.json").write_text(json.dumps(manifest))
        with (tmp_path / "metadata.mcap").open("wb") as output:
            writer = Writer(output, compression=CompressionType.NONE)
            writer.start()
            writer.add_metadata(
                "wrist.capture", {"media_mode": mode, "serial_number": "test-wrist"}
            )
            writer.add_metadata("wrist.calibration", {"json": json.dumps(calibration)})
            schema = writer.register_schema(
                "core.OrbbecWristFrameMetadataRecord", "flatbuffer", b"schema"
            )
            channel = writer.register_channel(
                "wrist_camera/Color", "flatbuffer", schema
            )
            writer.add_message(
                channel, 1000000, payload(encoded_data=encoded_data), 1000000
            )
            if mode == "embedded":
                channel = writer.register_channel(
                    "orbbec_wrist_media/Color", "flatbuffer", schema
                )
                writer.add_message(
                    channel, 1000000, payload(True, encoded_data), 1000000
                )
            channel = writer.register_channel("wrist_camera/Diagnostics", "json", 0)
            writer.add_message(
                channel,
                1000000,
                json.dumps({"state": 0 if closed else 2}).encode(),
                1000000,
            )
            writer.finish()
        return tmp_path

    return create


@pytest.mark.parametrize("mode", ["sidecar", "embedded"])
def test_delivery_compares_media_metadata_and_decoded_profile(recording, mode):
    run = recording(mode)
    probe = {
        "streams": [
            {
                "codec_name": "h264",
                "width": 1600,
                "height": 1200,
                "r_frame_rate": "30/1",
            }
        ]
    }
    results = [
        subprocess.CompletedProcess([], 0, "# framemd5\n0, frame\n", ""),
        subprocess.CompletedProcess([], 0, json.dumps(probe), ""),
    ]
    with patch.object(VERIFIER.subprocess, "run", side_effect=results):
        report = VERIFIER.verify(run)
    assert "PASS" in report
    assert "sha256=" in report
    assert "Decoded Color frames: 1" in report


def test_sidecar_size_mismatch_fails_before_decoder(recording):
    run = recording()
    (run / "raw/Color.h264").write_bytes(b"truncated")
    with pytest.raises(ValueError, match="size differs"):
        VERIFIER.verify(run)


def test_embedded_and_retained_media_must_match(recording):
    run = recording("embedded")
    (run / "raw/Color.h264").write_bytes(b"xxxx")
    with pytest.raises(ValueError, match="differs from embedded"):
        VERIFIER.verify(run)


def test_missing_final_close_event_fails(recording):
    with pytest.raises(ValueError, match="close event"):
        VERIFIER.verify(recording(closed=False))


def test_partial_mcap_and_required_calibration_fail(recording):
    run = recording()
    with pytest.raises(ValueError, match="calibration YAML"):
        VERIFIER.verify(run, require_calibration=True)
    (run / "metadata.mcap.partial").touch()
    with pytest.raises(ValueError, match="partial MCAP"):
        VERIFIER.verify(run)


def helper(*args):
    return subprocess.run(
        [
            "bash",
            str(PLUGIN / "orbbec_wrist.sh"),
            "record",
            "--plugin",
            "/bin/true",
            "--dry-run",
            *args,
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def test_helper_defaults_allow_auto_selection_and_sidecar_layout():
    result = helper()
    assert result.returncode == 0, result.stderr
    assert "/raw/Color.h264" in result.stdout
    assert "/metadata.mcap" in result.stdout
    assert "--media-mode sidecar" in result.stdout
    assert "--serial-number" not in result.stdout
    assert "--width" not in result.stdout
    assert "--height" not in result.stdout


def test_helper_embedded_mode_and_imu_override(tmp_path):
    config = tmp_path / "record.toml"
    config.write_text('[record]\nimu_rate = 400\nmedia_mode = "sidecar"\n')
    result = helper(
        "--imu-rate", "1000", "--config", str(config), "--media-mode", "embedded"
    )
    assert result.returncode == 0, result.stderr
    assert "--imu-rate 1000" in result.stdout
    assert "--video-output" not in result.stdout
    assert "--media-mode embedded" in result.stdout


@pytest.mark.parametrize("equals", [False, True])
@pytest.mark.parametrize("config_first", [False, True])
def test_helper_value_syntax_preserves_values_and_cli_precedence(
    tmp_path, equals, config_first
):
    config = tmp_path / "record = config.toml"
    config.write_text('[record]\nserial_number = "old"\nduration = 1\n')
    output = str(tmp_path / "run = 1")
    pairs = [
        ("--serial-number", "BM1V961000G = camera"),
        ("--duration", "60"),
        ("--output", output),
    ]
    cli = (
        [f"{key}={value}" for key, value in pairs]
        if equals
        else [item for pair in pairs for item in pair]
    )
    config_args = [f"--config={config}"] if equals else ["--config", str(config)]
    result = helper(*(config_args + cli if config_first else cli + config_args))
    assert result.returncode == 0, result.stderr
    arguments = shlex.split(result.stdout)
    assert arguments[arguments.index("--serial-number") + 1] == pairs[0][1]
    assert arguments[arguments.index("--duration") + 1] == "60"
    assert (
        arguments[arguments.index("--mcap-filename") + 1] == output + "/metadata.mcap"
    )
    assert not Path(output).exists()


@pytest.mark.parametrize(
    "args",
    [
        ("--serial-number",),
        ("--serial-number", ""),
        ("--serial-number=",),
        ("--config=",),
        ("--serial-number", "--preview"),
        ("--preview=true",),
        ("--unknown=value",),
    ],
)
def test_helper_rejects_missing_values_and_invalid_options(args):
    assert helper(*args).returncode != 0


def test_helper_rejects_duplicate_config_with_mixed_syntax(tmp_path):
    config = tmp_path / "record.toml"
    config.write_text("[record]\nduration = 1\n")
    result = helper("--config", str(config), f"--config={config}")
    assert result.returncode != 0
    assert "Specify --config only once" in result.stderr


@pytest.mark.parametrize(
    "args", [("--width", "1600"), ("--height", "1200"), ("--imu-rate", "500")]
)
def test_helper_rejects_removed_resolution_options_and_invalid_rate(args):
    assert helper(*args).returncode != 0


@pytest.mark.parametrize("dimension", ["width", "height"])
def test_helper_rejects_resolution_in_toml(tmp_path, dimension):
    config = tmp_path / "record.toml"
    config.write_text(f"[record]\n{dimension} = 1600\n")
    assert helper("--config", str(config)).returncode != 0


def test_failed_reverification_removes_stale_success_report(recording):
    run = recording()
    (run / "verification_report.txt").write_text("old PASS")
    (run / "raw/Color.h264").write_bytes(b"bad")
    result = subprocess.run(
        [sys.executable, str(PLUGIN / "tools/verify_recording.py"), str(run)],
        check=False,
        capture_output=True,
    )
    assert result.returncode != 0
    assert not (run / "verification_report.txt").exists()
    assert (run / "verification_report.failed.txt").exists()


@pytest.mark.skipif(
    not shutil.which("ffmpeg") or not shutil.which("ffprobe"), reason="FFmpeg required"
)
def test_record_helper_runs_complete_verification_with_real_encoded_video(
    recording, tmp_path
):
    encoded = tmp_path / "synthetic.h264"
    subprocess.run(
        [
            "ffmpeg",
            "-v",
            "error",
            "-f",
            "lavfi",
            "-i",
            "color=c=black:s=1600x1200:r=30",
            "-frames:v",
            "1",
            "-c:v",
            "libx264",
            "-threads",
            "1",
            str(encoded),
        ],
        check=True,
    )
    template = recording(encoded_data=encoded.read_bytes())
    output = tmp_path / "delivery"
    plugin = tmp_path / "fake_plugin.py"
    plugin.write_text(
        f"#!{sys.executable}\n"
        "import pathlib, shutil, sys\n"
        f"source = pathlib.Path({str(template)!r})\n"
        "argv = [arg for arg in sys.argv[1:] if arg != '--disable-imu']\n"
        "args = dict(zip(argv[::2], argv[1::2]))\n"
        "target = pathlib.Path(args['--mcap-filename']).parent\n"
        "for name in ('metadata.mcap', 'calibration.json', 'capabilities.txt'):\n"
        "    shutil.copyfile(source / name, target / name)\n"
        "shutil.copytree(source / 'raw', target / 'raw')\n"
        "print('mock capture finalized')\n"
    )
    plugin.chmod(0o755)
    result = subprocess.run(
        [
            "bash",
            str(PLUGIN / "orbbec_wrist.sh"),
            "record",
            "--plugin",
            str(plugin),
            "--output",
            str(output),
            "--duration",
            "1",
            "--no-imu",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "mock capture finalized" in (output / "logs/capture.log").read_text()
    assert "PASS" in (output / "verification_report.txt").read_text()
    assert (
        json.loads((output / "requested_profile.json").read_text())["streams"]["Color"][
            "width"
        ]
        == 1600
    )


@pytest.mark.parametrize("relative", [False, True])
def test_helper_passes_media_spool_from_cli_over_toml(tmp_path, relative):
    config = tmp_path / "spool.toml"
    config.write_text(
        '[record]\nmedia_mode = "embedded"\ncollection_prefix = "wrist/left"\nmcap_media_spool = "old.mcap"\n'
    )
    spool = "fragment.mcap" if relative else str(tmp_path / "fragment.mcap")
    result = helper("--config", str(config), "--mcap-media-spool", spool)
    assert result.returncode == 0, result.stderr
    assert "--mcap-media-spool " + spool in result.stdout
    assert "old.mcap" not in result.stdout
    assert "--mcap-filename " in result.stdout


@pytest.mark.parametrize(
    "options", [[], ["--media-mode", "embedded"], ["--collection-prefix", "wrist/left"]]
)
def test_helper_rejects_incomplete_media_spool_configuration(options):
    result = helper("--mcap-media-spool", "fragment.mcap", *options)
    assert result.returncode != 0
    assert "requires --media-mode embedded and --collection-prefix" in result.stderr
