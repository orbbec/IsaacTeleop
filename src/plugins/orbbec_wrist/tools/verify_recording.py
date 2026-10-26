# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Check Orbbec Wrist media against recorded metadata before producing a delivery report."""

import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile

from mcap.reader import make_reader


def unpack(data, offset, fmt):
    size = struct.calcsize(fmt)
    if offset < 0 or offset + size > len(data):
        raise ValueError("Truncated FlatBuffer")
    return struct.unpack_from(fmt, data, offset)


def field(data, table, index):
    vtable = table - unpack(data, table, "<i")[0]
    length = unpack(data, vtable, "<H")[0]
    if 4 + index * 2 + 2 > length:
        return None
    delta = unpack(data, vtable + 4 + index * 2, "<H")[0]
    return table + delta if delta else None


def scalar(data, table, index, fmt, default=0):
    position = field(data, table, index)
    return default if position is None else unpack(data, position, fmt)[0]


def record(data):
    root = unpack(data, 0, "<I")[0]
    position = field(data, root, 0)
    if position is None:
        raise ValueError("Record has no payload")
    return position + unpack(data, position, "<I")[0]


def vector(data, table, index, stride):
    position = field(data, table, index)
    if position is None:
        return 0, 0
    position += unpack(data, position, "<I")[0]
    count = unpack(data, position, "<I")[0]
    if position + 4 + count * stride > len(data):
        raise ValueError("Truncated vector")
    return position + 4, count


def fingerprint(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return f"{path}: bytes={path.stat().st_size} sha256={digest.hexdigest()}"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.digest()


def verify(run, require_calibration=False):
    run = Path(run).resolve()
    manifest = json.loads((run / "requested_profile.json").read_text())
    if manifest.get("version") != 1 or set(manifest["streams"]) != {"Color"}:
        raise ValueError("Expected a version 1 single-Color requested profile")
    request = manifest["streams"]["Color"]
    codecs = {"h264": (1, "h264"), "h265": (2, "hevc"), "mjpg": (0, "mjpeg")}
    codec, width, height, fps = (
        request[k] for k in ("format", "width", "height", "fps")
    )
    if (
        codec not in codecs
        or (width, height) != (1600, 1200)
        or type(fps) is not int
        or fps < 0
    ):
        raise ValueError("Invalid Orbbec Wrist requested profile")
    mode = manifest["media_mode"]
    if mode not in ("sidecar", "embedded"):
        raise ValueError("Invalid media mode")
    imu_request = manifest["imu"]
    if type(imu_request["enabled"]) is not bool or imu_request[
        "sample_rate_hz"
    ] not in (400, 1000):
        raise ValueError("Invalid IMU request")
    mcap_path = run / "metadata.mcap"
    if Path(str(mcap_path) + ".partial").exists():
        raise ValueError("Capture still has a partial MCAP")
    calibration = json.loads((run / "calibration.json").read_text())
    if require_calibration:
        color = calibration.get("color")
        if (
            not color
            or not calibration.get("raw_alignment_yaml")
            or not calibration.get("raw_imu_yaml")
        ):
            raise ValueError(
                "Complete Color intrinsics and raw calibration YAML are required"
            )
        if (color.get("width"), color.get("height")) != (1600, 1200):
            raise ValueError("Calibration resolution differs from the recording")
    for path in (run / "capabilities.txt", run / "logs/capture.log"):
        if not path.is_file() or not path.stat().st_size:
            raise ValueError(f"Missing capture evidence: {path}")
    frames, media = {}, {}
    imu_counts, gaps = Counter(), Counter()
    previous = {}
    final_state = None
    media_hash = hashlib.sha256()

    def continuity(stream, epoch, sequence, timestamp, rate=None):
        key = (stream, epoch)
        if timestamp <= 0:
            raise ValueError(f"{stream} has no device timestamp")
        if key in previous:
            last_sequence, last_time = previous[key]
            if sequence <= last_sequence or timestamp <= last_time:
                raise ValueError(
                    f"{stream} sequence/timestamp moves backward or repeats"
                )
            if rate:
                expected_us = (sequence - last_sequence) * 1000000 / rate
                if abs(timestamp - last_time - expected_us) > max(
                    2000, expected_us * 0.15
                ):
                    raise ValueError(
                        f"{stream} device timestamps differ from the declared frame rate"
                    )
            gaps[stream] += sequence - last_sequence - 1
        previous[key] = sequence, timestamp

    with tempfile.TemporaryDirectory(prefix="wrist-verify-") as temporary:
        extracted = Path(temporary) / f"Color.{codec}"
        with mcap_path.open("rb") as source, extracted.open("wb") as output:
            reader = make_reader(source, validate_crcs=True)
            capture_metadata = None
            recorded_calibration = None
            for item in reader.iter_metadata():
                if item.name == "wrist.capture":
                    capture_metadata = item.metadata
                elif item.name == "wrist.calibration":
                    recorded_calibration = json.loads(item.metadata["json"])
            if not capture_metadata or capture_metadata.get("media_mode") != mode:
                raise ValueError("MCAP media mode differs from the request")
            if recorded_calibration != calibration or capture_metadata.get(
                "serial_number"
            ) != calibration.get("serial_number"):
                raise ValueError(
                    "MCAP calibration/device identity differs from calibration.json"
                )
            if (
                manifest.get("serial_number")
                and manifest["serial_number"] != capture_metadata["serial_number"]
            ):
                raise ValueError(
                    "Captured serial number differs from the explicit request"
                )
            for schema, channel, message in reader.iter_messages(log_time_order=False):
                data = message.data
                if channel.topic == "wrist_camera/Diagnostics":
                    status = json.loads(data)
                    final_state = status["state"]
                    if final_state == 5:
                        raise ValueError(f"Capture fault: {status['error']}")
                    continue
                if not schema:
                    raise ValueError("Orbbec Wrist record has no schema")
                table = record(data)
                if channel.topic in ("wrist_camera/Color", "orbbec_wrist_media/Color"):
                    if scalar(data, table, 0, "<b") != 0:
                        raise ValueError("Unexpected Orbbec Wrist stream identity")
                    seq = scalar(data, table, 1, "<Q")
                    epoch = scalar(data, table, 7, "<I")
                    actual = tuple(
                        scalar(data, table, i, fmt)
                        for i, fmt in ((2, "<I"), (3, "<I"), (4, "<I"), (5, "<b"))
                    )
                    if (
                        actual[:2] != (1600, 1200)
                        or actual[3] != codecs[codec][0]
                        or actual[2] <= 0
                        or (fps and actual[2] != fps)
                    ):
                        raise ValueError(
                            f"Recorded Color profile differs from request: {actual}"
                        )
                    key = epoch, seq
                    if channel.topic == "wrist_camera/Color":
                        if key in frames:
                            raise ValueError("Repeated video metadata")
                        frames[key] = scalar(data, table, 6, "<Q")
                        continuity(
                            "Color", epoch, seq, scalar(data, table, 8, "<Q"), actual[2]
                        )
                    else:
                        if key in media:
                            raise ValueError("Repeated embedded video")
                        begin, count = vector(data, table, 6, 1)
                        payload = data[begin : begin + count]
                        output.write(payload)
                        media_hash.update(payload)
                        media[key] = count
                elif channel.topic in (
                    "wrist_camera/Accel/Imu",
                    "wrist_camera/Gyro/Imu",
                ):
                    sensor = (
                        "Accel" if channel.topic == "wrist_camera/Accel/Imu" else "Gyro"
                    )
                    if not imu_request["enabled"] or scalar(data, table, 0, "<b") != (
                        sensor == "Gyro"
                    ):
                        raise ValueError("Unexpected IMU sensor")
                    if scalar(data, table, 1, "<I") != imu_request["sample_rate_hz"]:
                        raise ValueError("IMU rate differs from request")
                    epoch = scalar(data, table, 3, "<I")
                    begin, count = vector(data, table, 4, 72)
                    for i in range(count):
                        sample = unpack(data, begin + i * 72, "<QddddQQqB")
                        if not all(math.isfinite(value) for value in sample[1:5]):
                            raise ValueError("Non-finite IMU data")
                        continuity(sensor, epoch, sample[0], sample[5])
                    imu_counts[sensor] += count
                else:
                    raise ValueError(f"Unexpected MCAP channel: {channel.topic}")
        if not frames or final_state != 0:
            raise ValueError(
                "Recording has no accepted video or no successful close event"
            )
        if imu_request["enabled"] and not all(
            imu_counts[sensor] for sensor in ("Accel", "Gyro")
        ):
            raise ValueError("Requested IMU sensor is empty")
        raw = run / "raw" / f"Color.{codec}"
        if mode == "embedded":
            if media != frames:
                raise ValueError("Embedded media does not match video metadata")
            video = extracted
            if raw.exists() and sha256(raw) != media_hash.digest():
                raise ValueError("Retained raw media differs from embedded media")
        else:
            if media or not raw.is_file() or raw.stat().st_size != sum(frames.values()):
                raise ValueError(
                    "Sidecar media size differs from metadata, or MCAP unexpectedly embeds video"
                )
            video = raw
        decoded = subprocess.run(
            [
                "ffmpeg",
                "-v",
                "error",
                "-xerror",
                "-err_detect",
                "explode",
                "-i",
                str(video),
                "-map",
                "0:v:0",
                "-f",
                "framemd5",
                "-",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        if decoded.stderr.strip():
            raise ValueError(f"Video decoding reported errors: {decoded.stderr}")
        decoded_count = sum(
            bool(line.strip()) and not line.startswith("#")
            for line in decoded.stdout.splitlines()
        )
        if decoded_count != len(frames):
            raise ValueError(
                f"Decoded frames {decoded_count} differ from metadata {len(frames)}"
            )
        probe = subprocess.run(
            ["ffprobe", "-v", "error", "-show_streams", "-of", "json", str(video)],
            check=True,
            capture_output=True,
            text=True,
        )
        streams = json.loads(probe.stdout)["streams"]
        if len(streams) != 1 or (
            streams[0]["codec_name"],
            streams[0]["width"],
            streams[0]["height"],
        ) != (codecs[codec][1], 1600, 1200):
            raise ValueError("Decoded media profile differs from request")
        if Fraction(streams[0]["r_frame_rate"]) <= 0:
            raise ValueError("Media has no valid video rate")
    lines = [
        "Orbbec Wrist recording verification: PASS",
        f"Directory: {run}",
        f"Serial: {capture_metadata['serial_number']}",
        f"Media mode: {mode}",
        f"Decoded Color frames: {len(frames)}",
        f"IMU samples: {dict(imu_counts)}",
        f"Sequence gaps (missing samples/frames): {dict(gaps)}",
        f"Calibration availability/errors: {calibration.get('errors', {})}",
    ]
    for path in (
        mcap_path,
        run / "calibration.json",
        run / "capabilities.txt",
        run / "requested_profile.json",
        run / "logs/capture.log",
    ):
        lines.append(fingerprint(path))
    if raw.is_file():
        lines.append(fingerprint(raw))
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--require-calibration", action="store_true")
    args = parser.parse_args()
    if not args.run_directory.is_dir():
        parser.error(f"Recording directory does not exist: {args.run_directory}")
    report = args.run_directory / "verification_report.txt"
    failed = args.run_directory / "verification_report.failed.txt"
    report.unlink(missing_ok=True)
    try:
        text = verify(args.run_directory, args.require_calibration)
    except Exception as error:
        failed.write_text(f"Orbbec Wrist recording verification: FAIL\n{error}\n")
        parser.exit(1, f"Verification failed: {error}\n")
    working = report.with_suffix(".txt.inprogress")
    working.write_text(text)
    working.replace(report)
    failed.unlink(missing_ok=True)
    print(text, end="")


if __name__ == "__main__":
    main()
