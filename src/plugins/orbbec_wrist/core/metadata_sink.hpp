// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "frame_sink.hpp"

#include <oxr_utils/oxr_session_handles.hpp>
#include <schema/orbbec_wrist_camera_generated.h>
#include <schema/orbbec_wrist_imu_generated.h>

namespace plugins::wrist
{
struct OutputConfig
{
    std::string collection_prefix;
    std::string mcap_filename;
    std::string mcap_media_spool;
    std::string plugin_root_id;
    std::string video_output;
    bool embedded_media = false;
};

core::OrbbecWristFrameMetadataT frame_payload(const VideoFrame& frame);
core::OrbbecWristEncodedVideoFrameT encoded_frame_payload(const VideoFrame& frame);
core::OrbbecWristImuBatchT imu_payload(const CaptureBatch& batch, ImuSensor sensor, const CaptureConfig& config);

void validate_output_config(const OutputConfig& output);

std::unique_ptr<FrameSink> create_frame_sink(const OutputConfig& output,
                                             const CaptureConfig& capture,
                                             const DeviceInfo& device);
std::unique_ptr<IMetadataSink> make_schema_metadata_sink(const OutputConfig& output, const CaptureConfig& capture);
// Borrowed handles must outlive the sink, including its publishing thread.
std::unique_ptr<IMetadataSink> make_schema_metadata_sink(const OutputConfig& output,
                                                         const CaptureConfig& capture,
                                                         const core::OpenXRSessionHandles& handles);
std::unique_ptr<IMetadataSink> make_mcap_metadata_sink(const std::string& filename,
                                                       const CaptureConfig& capture,
                                                       const DeviceInfo& device,
                                                       bool embedded_media = false,
                                                       const std::string& collection_prefix = {},
                                                       const std::string& plugin_root_id = {});
} // namespace plugins::wrist
