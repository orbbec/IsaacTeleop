// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "orbbec_wrist_camera.hpp"

namespace plugins::wrist
{
class IMetadataSink
{
public:
    virtual ~IMetadataSink() = default;
    // Arguments are borrowed; sinks must own any data retained after the call.
    virtual void on_frame_metadata(const VideoFrame& frame) = 0;
    virtual void on_encoded_video_frame(const VideoFrame&)
    {
    }
    virtual void on_imu_batch(const CaptureBatch&, ImuSensor)
    {
    }
    virtual void on_capture_status(CaptureState, const std::string&)
    {
    }
    virtual void close()
    {
    }
    virtual void commit()
    {
    }
    virtual void abort()
    {
        close();
    }
    virtual std::string error() const
    {
        return {};
    }
};

std::unique_ptr<IMetadataSink> compose_metadata_sinks(std::vector<std::unique_ptr<IMetadataSink>> sinks);

class FrameSink
{
public:
    FrameSink(const VideoProfile& profile,
              std::unique_ptr<IMetadataSink> metadata_sink = nullptr,
              const std::string& media_output = {});
    ~FrameSink();
    FrameSink(const FrameSink&) = delete;
    FrameSink& operator=(const FrameSink&) = delete;

    // The observer sees the same accepted bytes as recording and publication.
    void on_frame(const VideoFrame& frame, const std::function<void(const VideoFrame&)>& accepted_frame = {});
    void write(const CaptureBatch& batch, const std::function<void(const VideoFrame&)>& accepted_frame = {});
    void begin_capture_epoch();
    IMetadataSink* metadata_sink();
    void close_media();
    void close_metadata();
    void commit_metadata();
    void abort_metadata();
    std::string metadata_error() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace plugins::wrist
