// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>
#include <flatbuffers/flatbuffers.h>
#include <schema/ego_audio_generated.h>
#include <schema/ego_calibration_generated.h>
#include <schema/ego_camera_generated.h>
#include <schema/ego_device_state_generated.h>
#include <schema/ego_imu_generated.h>
#include <schema/timestamp_generated.h>

#include <cstdint>
#include <memory>

#define VT(field) (field + 2) * 2

static_assert(core::EgoFrameMetadata::VT_STREAM == VT(0));
static_assert(core::EgoFrameMetadata::VT_SEQUENCE_NUMBER == VT(1));
static_assert(core::EgoFrameMetadata::VT_WIDTH == VT(2));
static_assert(core::EgoFrameMetadata::VT_HEIGHT == VT(3));
static_assert(core::EgoFrameMetadata::VT_FPS == VT(4));
static_assert(core::EgoFrameMetadata::VT_PIXEL_FORMAT == VT(5));
static_assert(core::EgoFrameMetadata::VT_ENCODED_BYTES == VT(6));
static_assert(core::EgoFrameMetadata::VT_SDK_METADATA == VT(7));
static_assert(core::EgoFrameMetadata::VT_CAPTURE_EPOCH == VT(8));
static_assert(core::EgoPixelFormat_Mjpg == 0);

TEST_CASE("EGO camera metadata round trips", "[unit][ego][schema]")
{
    core::EgoFrameMetadataT original;
    original.stream = core::EgoCameraStream_ColorRight;
    original.sequence_number = UINT64_MAX;
    original.width = 1280;
    original.height = 720;
    original.fps = 30;
    original.pixel_format = core::EgoPixelFormat_Mjpg;
    original.encoded_bytes = 1234;
    original.sdk_metadata.emplace_back(1, 99);
    original.capture_epoch = 3;

    flatbuffers::FlatBufferBuilder builder;
    builder.Finish(core::EgoFrameMetadata::Pack(builder, &original));

    core::EgoFrameMetadataT restored;
    flatbuffers::GetRoot<core::EgoFrameMetadata>(builder.GetBufferPointer())->UnPackTo(&restored);
    CHECK(restored.stream == core::EgoCameraStream_ColorRight);
    CHECK(restored.sequence_number == UINT64_MAX);
    CHECK(restored.width == 1280);
    CHECK(restored.height == 720);
    CHECK(restored.fps == 30);
    CHECK(restored.pixel_format == core::EgoPixelFormat_Mjpg);
    CHECK(restored.encoded_bytes == 1234);
    REQUIRE(restored.sdk_metadata.size() == 1);
    CHECK(restored.sdk_metadata[0].value() == 99);
    CHECK(restored.capture_epoch == 3);
}

TEST_CASE("EGO auxiliary schemas round trip", "[unit][ego][schema]")
{
    core::EgoImuBatchT imu;
    imu.sensor = core::EgoImuSensor_Gyro;
    imu.sequence_number = 5;
    imu.sample_rate_hz = 1000;
    imu.full_scale = 500;
    imu.samples.emplace_back(1, 2, 3, 24, 100, 200);
    imu.capture_epoch = 2;
    flatbuffers::FlatBufferBuilder imu_builder;
    imu_builder.Finish(core::EgoImuBatch::Pack(imu_builder, &imu));
    core::EgoImuBatchT restored_imu;
    flatbuffers::GetRoot<core::EgoImuBatch>(imu_builder.GetBufferPointer())->UnPackTo(&restored_imu);
    REQUIRE(restored_imu.samples.size() == 1);
    CHECK(restored_imu.samples[0].z_si() == 3);
    CHECK(restored_imu.capture_epoch == 2);

    core::EgoAudioChunkT audio;
    audio.sample_rate_hz = 48000;
    audio.channel_count = 1;
    audio.bits_per_sample = 16;
    audio.sample_count = 480;
    audio.wav_data_offset = 44;
    audio.capture_epoch = 2;
    flatbuffers::FlatBufferBuilder audio_builder;
    audio_builder.Finish(core::EgoAudioChunk::Pack(audio_builder, &audio));
    core::EgoAudioChunkT restored_audio;
    flatbuffers::GetRoot<core::EgoAudioChunk>(audio_builder.GetBufferPointer())->UnPackTo(&restored_audio);
    CHECK(restored_audio.wav_data_offset == 44);
    CHECK(restored_audio.sample_count == 480);
    CHECK(restored_audio.capture_epoch == 2);

    core::EgoCalibrationT calibration;
    calibration.device_uid = "ego";
    calibration.raw_alignment_yaml = "camera: left";
    calibration.capture_epoch = 2;
    flatbuffers::FlatBufferBuilder calibration_builder;
    calibration_builder.Finish(core::EgoCalibration::Pack(calibration_builder, &calibration));
    core::EgoCalibrationT restored_calibration;
    flatbuffers::GetRoot<core::EgoCalibration>(calibration_builder.GetBufferPointer())->UnPackTo(&restored_calibration);
    CHECK(restored_calibration.device_uid == "ego");
    CHECK(restored_calibration.capture_epoch == 2);

    core::EgoDeviceStateT state;
    state.sequence_number = 7;
    state.temperature_c = 31.5f;
    state.capture_health = core::EgoCaptureHealth_Warning;
    state.failure_reason = "queue warning";
    state.queue_capacity = 4096;
    state.queue_peak = 3482;
    state.capture_epoch = 2;
    state.connection_state = core::EgoConnectionState_Recovered;
    state.reconnect_attempt = 4;
    state.properties.emplace_back(279, 8'000'000);
    flatbuffers::FlatBufferBuilder state_builder;
    state_builder.Finish(core::EgoDeviceState::Pack(state_builder, &state));
    core::EgoDeviceStateT restored_state;
    flatbuffers::GetRoot<core::EgoDeviceState>(state_builder.GetBufferPointer())->UnPackTo(&restored_state);
    CHECK(restored_state.temperature_c == 31.5f);
    CHECK(restored_state.capture_health == core::EgoCaptureHealth_Warning);
    CHECK(restored_state.failure_reason == "queue warning");
    CHECK(restored_state.queue_capacity == 4096);
    CHECK(restored_state.capture_epoch == 2);
    CHECK(restored_state.connection_state == core::EgoConnectionState_Recovered);
    CHECK(restored_state.reconnect_attempt == 4);
    REQUIRE(restored_state.properties.size() == 1);
}

TEST_CASE("EGO camera record retains timestamp", "[unit][ego][schema]")
{
    auto record = std::make_shared<core::EgoFrameMetadataRecordT>();
    record->data = std::make_shared<core::EgoFrameMetadataT>();
    record->timestamp = std::make_shared<core::DeviceDataTimestamp>(1, 2, 3);

    flatbuffers::FlatBufferBuilder builder;
    builder.Finish(core::EgoFrameMetadataRecord::Pack(builder, record.get()));
    const auto* restored = flatbuffers::GetRoot<core::EgoFrameMetadataRecord>(builder.GetBufferPointer());
    REQUIRE(restored->timestamp() != nullptr);
    CHECK(restored->timestamp()->available_time_local_common_clock() == 1);
    CHECK(restored->timestamp()->sample_time_local_common_clock() == 2);
    CHECK(restored->timestamp()->sample_time_raw_device_clock() == 3);
}

TEST_CASE("EGO embedded media schemas round trip", "[unit][ego][schema]")
{
    core::EgoEncodedVideoFrameT video;
    video.stream = core::EgoCameraStream_ColorRight;
    video.sequence_number = 42;
    video.width = 1600;
    video.height = 1300;
    video.fps = 30;
    video.pixel_format = core::EgoPixelFormat_H264;
    video.encoded_data = { 0, 0, 0, 1, 0x65, 1 };
    video.capture_epoch = 2;
    flatbuffers::FlatBufferBuilder video_builder;
    video_builder.Finish(core::EgoEncodedVideoFrame::Pack(video_builder, &video));
    core::EgoEncodedVideoFrameT restored_video;
    flatbuffers::GetRoot<core::EgoEncodedVideoFrame>(video_builder.GetBufferPointer())->UnPackTo(&restored_video);
    CHECK(restored_video.stream == core::EgoCameraStream_ColorRight);
    CHECK(restored_video.encoded_data == video.encoded_data);
    CHECK(restored_video.capture_epoch == 2);

    core::EgoPcmAudioChunkT audio;
    audio.sequence_number = 7;
    audio.sample_rate_hz = 48000;
    audio.channel_count = 1;
    audio.bits_per_sample = 16;
    audio.sample_count = 2;
    audio.pcm_data = { 1, 0, 2, 0 };
    audio.capture_epoch = 2;
    flatbuffers::FlatBufferBuilder audio_builder;
    audio_builder.Finish(core::EgoPcmAudioChunk::Pack(audio_builder, &audio));
    core::EgoPcmAudioChunkT restored_audio;
    flatbuffers::GetRoot<core::EgoPcmAudioChunk>(audio_builder.GetBufferPointer())->UnPackTo(&restored_audio);
    CHECK(restored_audio.pcm_data == audio.pcm_data);
    CHECK(restored_audio.capture_epoch == 2);
}
