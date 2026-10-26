# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Orbbec Wrist mono preview; the producer owns capture, decoding and SDK cleanup."""

from __future__ import annotations

import logging
import re
import threading
import time
from typing import Optional

import numpy as np

from pipeline import Frame, FrameSource, SourceSpec
from ._helpers import alloc_pinned_host, notify, notify_verbose
from ._nv_decode import NvVideoDecoder

logger = logging.getLogger("isaaccapture.camera_viz.sources.orbbec_wrist")


# Keep Orbbec Wrist recovery local to avoid changing other camera implementations.
def _nal_units(packet: bytes, hevc: bool):
    starts = list(re.finditer(rb"\x00\x00\x00?\x01", packet))
    for index, start in enumerate(starts):
        end = starts[index + 1].start() if index + 1 < len(starts) else len(packet)
        if start.end() + (2 if hevc else 1) > end:
            continue
        header = packet[start.end()]
        nal_type = (header >> 1) & 0x3F if hevc else header & 0x1F
        yield nal_type, packet[start.start() : end]


class EncodedVideoDecoder:
    """Keep Orbbec Wrist NVDEC references valid across capture gaps."""

    def __init__(
        self,
        width: int,
        height: int,
        *,
        gpu_id: int,
        codec: str,
        decoder_factory=NvVideoDecoder,
    ):
        self._decoder = decoder_factory(width, height, gpu_id=gpu_id, codec=codec)
        self._hevc = codec == "h265"
        self._parameter_types = (32, 33, 34) if self._hevc else (7, 8)
        self._sequence = None
        self._waiting = True
        self._parameters = {}
        self._skip_rasl = False

    def reset(self) -> None:
        self._decoder.reset()
        self._sequence = None
        self._waiting = True
        self._parameters.clear()
        self._skip_rasl = False

    def decode(self, packet: bytes, sequence: int, rgba_out) -> bool:
        if self._sequence is not None and sequence != self._sequence + 1:
            self.reset()
        # SEI-only and deliberately skipped packets still consume SDK sequence numbers.
        self._sequence = sequence
        if self._waiting or self._skip_rasl:
            units = list(_nal_units(packet, self._hevc))
            types = {nal_type for nal_type, _ in units}
            if self._waiting:
                self._parameters.update(
                    (nal_type, data)
                    for nal_type, data in units
                    if nal_type in self._parameter_types
                )
                keyframes = {19, 20, 21} if self._hevc else {5}
                if not types.intersection(keyframes) or not all(
                    nal_type in self._parameters for nal_type in self._parameter_types
                ):
                    return False
                # Replay the full dependency order when recovery parameters span packets.
                if not all(nal_type in types for nal_type in self._parameter_types):
                    packet = (
                        b"".join(
                            self._parameters[nal_type]
                            for nal_type in self._parameter_types
                        )
                        + packet
                    )
                self._parameters.clear()
                self._waiting = False
                self._skip_rasl = self._hevc and 21 in types
            elif types.intersection({8, 9}):
                # CRA leading RASL pictures may reference pictures before random access.
                return False
            elif any(nal_type <= 31 and nal_type not in (6, 7) for nal_type in types):
                self._skip_rasl = False
        return self._decoder.decode(packet, rgba_out)


class OrbbecWristSource(FrameSource):
    """One Orbbec Wrist Color stream producing GPU RGBA8 frames."""

    def __init__(
        self,
        name: str,
        serial_number: str = "",
        width: int = 1600,
        height: int = 1200,
        fps: int = 30,
        format: str = "h264",
        gpu_id: int = 0,
        *,
        renderer_cuda_device_id: int | None = None,
    ) -> None:
        if (width, height) != (1600, 1200) or fps <= 0:
            raise ValueError("Orbbec Wrist requires 1600x1200 and positive fps")
        self._spec = SourceSpec(name=name, width=width, height=height)
        self._serial_number = serial_number
        self._fps = fps
        self._format = format.lower()
        if self._format not in ("mjpg", "h264", "h265"):
            raise ValueError("Orbbec Wrist format must be mjpg, h264, or h265")
        self._requested_gpu_id = gpu_id
        self._renderer_cuda_device_id = renderer_cuda_device_id
        self._gpu_id = gpu_id
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._error: Optional[BaseException] = None
        self._publish_idx = -1
        self._consumed_idx = -2
        self._timestamp_ns = 0
        self._write_idx = 0
        self._buffers = []
        self._capture = None
        self._skipped_frames = 0
        self._frames = 0

    @property
    def spec(self) -> SourceSpec:
        return self._spec

    def start(self) -> None:
        if self._thread is not None:
            return
        gpu_id = self._requested_gpu_id
        renderer_gpu = self._renderer_cuda_device_id
        if gpu_id < 0 or (renderer_gpu is not None and renderer_gpu < 0):
            raise ValueError("Orbbec Wrist requires a nonnegative CUDA device ID")
        if renderer_gpu is not None:
            # Zero retains the source's auto-selection contract, not a pin to GPU 0.
            if gpu_id != 0 and gpu_id != renderer_gpu:
                raise ValueError(
                    f"Orbbec Wrist source {self._spec.name!r} requests CUDA GPU {gpu_id}, "
                    f"but the renderer uses CUDA GPU {renderer_gpu}; "
                    "set gpu_id to 0 to follow the renderer"
                )
            gpu_id = renderer_gpu
        try:
            import cupy as cp
            from orbbec_wrist_preview import Capture  # noqa: F401
        except ImportError as exc:
            raise RuntimeError(
                "OrbbecWristSource requires CuPy and the native Orbbec Wrist binding. "
                "Run `camera_viz.sh setup --with-orbbec-wrist "
                "--sdk-root PATH`."
            ) from exc
        if renderer_gpu is None and gpu_id == 0:
            gpu_id = int(cp.cuda.runtime.getDevice())
        self._gpu_id = gpu_id
        with self._lock:
            self._error = None
            self._publish_idx = -1
            self._consumed_idx = -2
            self._timestamp_ns = 0
        self._write_idx = 0
        self._skipped_frames = 0
        self._frames = 0
        with cp.cuda.Device(self._gpu_id):
            buffers = [
                cp.empty((self._spec.height, self._spec.width, 4), dtype=cp.uint8)
                for _ in range(3)
            ]
            for buffer in buffers:
                buffer[..., 3] = 255
        self._buffers = buffers
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._produce_loop,
            name=f"orbbec_wrist_{self._spec.name}",
            daemon=False,
        )
        try:
            self._thread.start()
        except Exception:
            # An interrupt may arrive after the thread starts; retain it for stop().
            self._thread = None
            self._buffers = []
            raise

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                # Match NV's source stop budget; a live SDK call owns its handle.
                logger.warning(
                    "Orbbec Wrist producer did not exit within 5s; retaining SDK handle"
                )
                return
            self._thread = None
        self._capture = None

    def latest(self) -> Optional[Frame]:
        with self._lock:
            if self._error is not None:
                raise RuntimeError(
                    f"Orbbec Wrist source {self._spec.name!r} failed: {self._error}"
                ) from self._error
            if self._publish_idx < 0 or self._publish_idx == self._consumed_idx:
                return None
            index = self._publish_idx
            self._consumed_idx = index
            timestamp_ns = self._timestamp_ns
        return Frame(
            image=self._buffers[index],
            timestamp_ns=timestamp_ns,
            source_id=self._spec.name,
            stream=0,
        )

    def _produce_loop(self) -> None:
        try:
            self._capture_loop()
        except BaseException as exc:
            # Report before SDK cleanup, which may block while latest() is polled.
            self._record_error(exc)
            logger.exception(
                "Orbbec Wrist producer stopped after a capture or decode failure"
            )
        finally:
            try:
                self._close_capture()
            except BaseException as exc:
                self._record_error(exc)
                logger.exception("Orbbec Wrist SDK close failed")

    def _record_error(self, exc: BaseException) -> None:
        with self._lock:
            # A cleanup failure must not replace the original producer failure.
            if self._error is None:
                self._error = exc
        self._stop.set()

    def _close_capture(self) -> None:
        capture, self._capture = self._capture, None
        if capture is not None:
            capture.close()

    def _capture_loop(self) -> None:
        import cupy as cp
        from orbbec_wrist_preview import Capture

        with cp.cuda.Device(self._gpu_id):
            stream = cp.cuda.Stream(non_blocking=True)
            staging = landing = decoder = None
            if self._format == "mjpg":
                shape = (self._spec.height, self._spec.width, 3)
                staging = alloc_pinned_host(shape, np.uint8)
                landing = cp.empty(shape, dtype=cp.uint8)
            else:
                decoder = EncodedVideoDecoder(
                    self._spec.width,
                    self._spec.height,
                    gpu_id=self._gpu_id,
                    codec=self._format,
                )

            last_report = time.monotonic()
            while not self._stop.is_set():
                if self._capture is None:
                    try:
                        notify("orbbec_wrist", "opening...")
                        self._capture = Capture(
                            self._serial_number,
                            self._spec.width,
                            self._spec.height,
                            self._fps,
                            self._format,
                        )
                        self._serial_number = self._capture.serial_number
                        notify(
                            "orbbec_wrist", f"connected serial={self._serial_number}"
                        )
                    except Exception as exc:
                        notify(
                            "orbbec_wrist",
                            f"open failed ({exc}); retrying",
                            logging.WARNING,
                        )
                        self._stop.wait(5.0)
                        continue
                try:
                    frame = self._capture.next_frame(100)
                except Exception as exc:
                    notify(
                        "orbbec_wrist",
                        f"capture failed ({exc}); reconnecting",
                        logging.WARNING,
                    )
                    self._close_capture()
                    if decoder:
                        decoder.reset()
                    self._stop.wait(1.0)
                    continue
                if frame is None:
                    self._skipped_frames += 1
                    continue

                index = self._write_idx
                if self._format == "mjpg":
                    np.copyto(
                        staging,
                        np.frombuffer(frame["data"], dtype=np.uint8).reshape(
                            self._spec.height, self._spec.width, 3
                        ),
                    )
                    with stream:
                        landing.set(staging)
                        self._buffers[index][..., :3] = landing
                    stream.synchronize()
                    produced = True
                else:
                    produced = decoder.decode(
                        frame["data"], int(frame["sequence"]), self._buffers[index]
                    )
                if not produced:
                    self._skipped_frames += 1
                    continue
                with self._lock:
                    self._publish_idx = index
                    self._timestamp_ns = int(frame["timestamp_ns"])
                self._write_idx = (index + 1) % len(self._buffers)
                self._frames += 1
                if self._frames == 1:
                    notify("orbbec_wrist", "streaming")

                now = time.monotonic()
                if now - last_report >= 5.0:
                    notify_verbose(
                        "orbbec_wrist",
                        f"frames={self._frames} skipped={self._skipped_frames}",
                    )
                    last_report = now
