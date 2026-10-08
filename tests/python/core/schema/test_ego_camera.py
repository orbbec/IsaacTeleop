# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import gc

import pytest

from isaaccapture.schema import (
    DeviceDataTimestamp,
    EgoFrameMetadata,
    EgoFrameMetadataRecord,
    EgoAudioChunk,
    EgoAudioChunkRecord,
    EgoCalibration,
    EgoCalibrationRecord,
    EgoCameraIntrinsics,
    EgoCameraStream,
    EgoCaptureHealth,
    EgoConnectionState,
    EgoDevicePropertyValue,
    EgoDeviceState,
    EgoDeviceStateRecord,
    EgoEncodedVideoFrame,
    EgoEncodedVideoFrameRecord,
    EgoExtrinsics,
    EgoFrameMetadataEntry,
    EgoImuBatch,
    EgoImuBatchRecord,
    EgoImuSample,
    EgoImuSensor,
    EgoPcmAudioChunk,
    EgoPcmAudioChunkRecord,
    EgoPixelFormat,
)


def test_camera_payload_and_retained_record_views():
    metadata = EgoFrameMetadata(
        stream=EgoCameraStream.ColorRight,
        sequence_number=2**64 - 1,
        width=1600,
        height=1300,
        fps=30,
        pixel_format=EgoPixelFormat.H265,
        encoded_bytes=123,
        sdk_metadata=[EgoFrameMetadataEntry(1, 99)],
        capture_epoch=3,
    )
    assert metadata.stream == EgoCameraStream.ColorRight
    assert metadata.sequence_number == 2**64 - 1
    assert metadata.sdk_metadata[0].value == 99
    assert "ColorRight" in repr(metadata)
    with pytest.raises(AttributeError):
        metadata.capture_epoch = 4

    record = EgoFrameMetadataRecord(metadata, DeviceDataTimestamp(1, 2, 3))
    payload = record.data
    timestamp = record.timestamp
    del metadata, record
    gc.collect()
    assert payload.capture_epoch == 3
    assert timestamp.sample_time_raw_device_clock == 3
    assert EgoFrameMetadataRecord(None, DeviceDataTimestamp(1, 2, 3)).data is None


def test_auxiliary_payloads_and_nested_calibration():
    imu = EgoImuBatch(
        sensor=EgoImuSensor.Gyro,
        sample_rate_hz=1000,
        samples=[EgoImuSample(1, 2, 3, 24, 100, 200)],
        capture_epoch=2,
    )
    assert imu.samples[0].z_si == 3
    assert imu.samples[0].sample_time_raw_device_clock_ns == 200
    audio = EgoAudioChunk(sample_rate_hz=48000, wav_data_offset=44, capture_epoch=2)
    assert audio.wav_data_offset == 44
    intrinsics = EgoCameraIntrinsics(width=1600, fx=800, distortion=[1, 2])
    calibration = EgoCalibration(
        device_uid="ego",
        color_left=intrinsics,
        left_to_right=EgoExtrinsics(rotation=[1, 0, 0], translation_mm=[60, 0, 0]),
        capture_epoch=2,
    )
    left = calibration.color_left
    del calibration, intrinsics
    gc.collect()
    assert left.width == 1600
    assert left.distortion == [1, 2]
    assert EgoCalibration().color_left is None

    state = EgoDeviceState(
        capture_health=EgoCaptureHealth.Warning,
        failure_reason="queue warning",
        queue_capacity=4096,
        capture_epoch=2,
        connection_state=EgoConnectionState.Recovered,
        reconnect_attempt=4,
        properties=[EgoDevicePropertyValue(279, 8_000_000)],
    )
    assert state.properties[0].property_id == 279
    assert state.capture_health == EgoCaptureHealth.Warning
    assert state.connection_state == EgoConnectionState.Recovered
    assert state.reconnect_attempt == 4


def test_embedded_video_and_pcm_record_views():
    video = EgoEncodedVideoFrame(
        stream=EgoCameraStream.ColorRight,
        encoded_data=[0, 0, 0, 1, 0x65],
        capture_epoch=2,
    )
    audio = EgoPcmAudioChunk(
        sample_rate_hz=48000, pcm_data=[1, 0, 2, 0], capture_epoch=2
    )
    timestamp = DeviceDataTimestamp(100, 90, 3)
    video_record = EgoEncodedVideoFrameRecord(video, timestamp)
    audio_record = EgoPcmAudioChunkRecord(audio, timestamp)
    assert video_record.data.encoded_data == [0, 0, 0, 1, 0x65]
    assert audio_record.data.pcm_data == [1, 0, 2, 0]
    assert audio_record.timestamp.sample_time_local_common_clock == 90


def test_no_tracked_wrapper_compatibility_layer():
    import isaaccapture.schema as schema

    assert not hasattr(schema, "EgoFrameMetadataTrackedT")
    assert not hasattr(schema, "EgoAudioChunkTrackedT")


@pytest.mark.parametrize(
    ("payload_type", "record_type"),
    [
        (EgoFrameMetadata, EgoFrameMetadataRecord),
        (EgoImuBatch, EgoImuBatchRecord),
        (EgoAudioChunk, EgoAudioChunkRecord),
        (EgoCalibration, EgoCalibrationRecord),
        (EgoDeviceState, EgoDeviceStateRecord),
        (EgoEncodedVideoFrame, EgoEncodedVideoFrameRecord),
        (EgoPcmAudioChunk, EgoPcmAudioChunkRecord),
    ],
)
def test_every_record_maps_absence_and_retains_immutable_payload(
    payload_type, record_type
):
    timestamp = DeviceDataTimestamp(100, 90, 80)
    assert record_type(None, timestamp).data is None
    record = record_type(payload_type(capture_epoch=3), timestamp)
    payload = record.data
    del record
    gc.collect()
    assert payload.capture_epoch == 3
    assert not hasattr(payload, "data")
    with pytest.raises(AttributeError):
        payload.capture_epoch = 4
