// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "metadata_sink.hpp"

#include <log_bridge/logger.hpp>

#include <filesystem>
#include <set>
#include <utility>

namespace plugins::wrist
{
core::OrbbecWristFrameMetadataT frame_payload(const VideoFrame& frame)
{
    core::OrbbecWristFrameMetadataT result;
    result.stream = core::OrbbecWristCameraStream_Color;
    switch (frame.profile.format)
    {
    case PixelFormat::Mjpg:
        result.pixel_format = core::OrbbecWristPixelFormat_Mjpg;
        break;
    case PixelFormat::H264:
        result.pixel_format = core::OrbbecWristPixelFormat_H264;
        break;
    case PixelFormat::H265:
        result.pixel_format = core::OrbbecWristPixelFormat_H265;
        break;
    default:
        throw CaptureError(ErrorCode::MalformedFrame, "Unsupported Orbbec Wrist frame format");
    }
    result.sequence_number = frame.sequence;
    result.width = frame.profile.width;
    result.height = frame.profile.height;
    result.fps = frame.profile.fps;
    result.encoded_bytes = frame.encoded_data.size();
    result.capture_epoch = frame.capture_epoch;
    result.device_timestamp_us = frame.time.device_us;
    result.system_timestamp_us = frame.time.system_us;
    result.retrieved_monotonic_ns = frame.time.retrieved_monotonic_ns;
    result.discontinuity = frame.discontinuity;
    return result;
}

core::OrbbecWristEncodedVideoFrameT encoded_frame_payload(const VideoFrame& frame)
{
    const auto metadata = frame_payload(frame);
    core::OrbbecWristEncodedVideoFrameT result;
    result.stream = metadata.stream;
    result.sequence_number = metadata.sequence_number;
    result.width = metadata.width;
    result.height = metadata.height;
    result.fps = metadata.fps;
    result.pixel_format = metadata.pixel_format;
    result.encoded_data = frame.encoded_data;
    result.capture_epoch = metadata.capture_epoch;
    return result;
}

core::OrbbecWristImuBatchT imu_payload(const CaptureBatch& batch, ImuSensor sensor, const CaptureConfig& config)
{
    core::OrbbecWristImuBatchT result;
    result.sensor = sensor == ImuSensor::Accel ? core::OrbbecWristImuSensor_Accel : core::OrbbecWristImuSensor_Gyro;
    result.sample_rate_hz = config.imu_rate_hz;
    result.full_scale = sensor == ImuSensor::Accel ? config.accel_full_scale_g : config.gyro_full_scale_dps;
    for (const auto& sample : batch.imu)
    {
        if (sample.sensor != sensor)
            continue;
        if (!result.samples.empty() && sample.capture_epoch != result.capture_epoch)
            throw CaptureError(ErrorCode::MalformedFrame, "Mixed Orbbec Wrist epochs in IMU batch");
        result.capture_epoch = sample.capture_epoch;
        result.samples.emplace_back(sample.sequence, sample.value_si[0], sample.value_si[1], sample.value_si[2],
                                    sample.temperature_c, sample.time.device_us, sample.time.system_us,
                                    sample.time.retrieved_monotonic_ns, sample.discontinuity);
    }
    return result;
}

namespace
{
class CompositeMetadataSink final : public IMetadataSink
{
public:
    explicit CompositeMetadataSink(std::vector<std::unique_ptr<IMetadataSink>> sinks) : sinks_(std::move(sinks))
    {
    }
    void on_frame_metadata(const VideoFrame& frame) override
    {
        for_each([&](auto& sink) { sink.on_frame_metadata(frame); });
    }
    void on_encoded_video_frame(const VideoFrame& frame) override
    {
        for_each([&](auto& sink) { sink.on_encoded_video_frame(frame); });
    }
    void on_imu_batch(const CaptureBatch& batch, ImuSensor sensor) override
    {
        for_each([&](auto& sink) { sink.on_imu_batch(batch, sensor); });
    }
    void on_capture_status(CaptureState state, const std::string& error) override
    {
        for_each([&](auto& sink) { sink.on_capture_status(state, error); });
    }
    void close() override
    {
        closed_successfully_ = false;
        finish([](auto& sink) { sink.close(); });
        closed_successfully_ = true;
    }
    void commit() override
    {
        if (aborted_ || !closed_successfully_)
            throw std::runtime_error("Cannot commit Orbbec Wrist metadata before every sink closes successfully");
        for_each([](auto& sink) { sink.commit(); });
    }
    void abort() override
    {
        aborted_ = true;
        closed_successfully_ = false;
        finish([](auto& sink) { sink.abort(); });
    }
    std::string error() const override
    {
        for (const auto& sink : sinks_)
            if (!sink->error().empty())
                return sink->error();
        return {};
    }

private:
    template <typename Finish>
    void finish(Finish&& finish_sink)
    {
        std::string failures;
        // Seal local archives first; promotion waits for every sink to close.
        for (size_t offset = 0; offset < sinks_.size(); ++offset)
        {
            const size_t index = sinks_.size() - offset - 1;
            try
            {
                finish_sink(*sinks_[index]);
            }
            catch (const std::exception& error)
            {
                if (!failures.empty())
                    failures += "; ";
                failures += "sink " + std::to_string(index) + ": " + error.what();
            }
            catch (...)
            {
                if (!failures.empty())
                    failures += "; ";
                failures += "sink " + std::to_string(index) + ": unknown error";
            }
        }
        if (!failures.empty())
            throw std::runtime_error(failures);
    }
    template <typename Function>
    void for_each(Function&& function)
    {
        for (auto& sink : sinks_)
            function(*sink);
    }
    bool closed_successfully_ = false;
    std::vector<std::unique_ptr<IMetadataSink>> sinks_;
    bool aborted_ = false;
};
} // namespace

std::unique_ptr<IMetadataSink> compose_metadata_sinks(std::vector<std::unique_ptr<IMetadataSink>> sinks)
{
    return std::make_unique<CompositeMetadataSink>(std::move(sinks));
}

void validate_output_config(const OutputConfig& output)
{
    if (!output.mcap_media_spool.empty() && (!output.embedded_media || output.collection_prefix.empty()))
        throw std::invalid_argument(
            "Orbbec Wrist --mcap-media-spool requires --media-mode=embedded and --collection-prefix");
    if (!output.mcap_media_spool.empty() &&
        (output.collection_prefix.find_first_not_of('/') == std::string::npos ||
         output.collection_prefix == "session_observed" || output.collection_prefix.starts_with("session_observed/")))
        throw std::invalid_argument("Orbbec Wrist media spool requires a nonempty, non-reserved collection prefix");
    if (output.embedded_media && output.mcap_filename.empty() && output.mcap_media_spool.empty())
        throw std::invalid_argument("Orbbec Wrist embedded media requires --mcap-filename or --mcap-media-spool");
    std::set<std::filesystem::path> paths;
    const auto require_new_output = [&](const std::string& name)
    {
        if (name.empty())
            return;
        const std::filesystem::path path(name);
        if (std::filesystem::exists(path) || std::filesystem::is_symlink(path))
            throw std::invalid_argument("Orbbec Wrist output already exists: " + name);
        if (!paths.insert(std::filesystem::weakly_canonical(std::filesystem::absolute(path))).second)
            throw std::invalid_argument("Orbbec Wrist outputs refer to the same path: " + name);
    };
    require_new_output(output.video_output);
    require_new_output(output.mcap_filename);
    if (!output.mcap_filename.empty())
        require_new_output(output.mcap_filename + ".partial");
    require_new_output(output.mcap_media_spool);
    if (!output.mcap_media_spool.empty())
        require_new_output(output.mcap_media_spool + ".partial");
}

std::unique_ptr<FrameSink> create_frame_sink(const OutputConfig& output,
                                             const CaptureConfig& capture,
                                             const DeviceInfo& device)
{
    validate_output_config(output);
    std::vector<std::unique_ptr<IMetadataSink>> sinks;
    if (!output.collection_prefix.empty())
        sinks.push_back(make_schema_metadata_sink(output, capture));
    else if (!output.plugin_root_id.empty())
    {
        try
        {
            sinks.push_back(make_schema_metadata_sink(output, capture));
        }
        catch (const std::exception& error)
        {
            isaaccapture::Logger::get("isaaccapture.plugins.orbbec_wrist.FrameSink")
                ->warn("Orbbec Wrist device status session unavailable; local capture continues: {}", error.what());
        }
    }
    if (!output.mcap_filename.empty())
        sinks.push_back(make_mcap_metadata_sink(output.mcap_filename, capture, device, output.embedded_media));
    if (!output.mcap_media_spool.empty())
        sinks.push_back(make_mcap_metadata_sink(
            output.mcap_media_spool, capture, device, true, output.collection_prefix, output.plugin_root_id));
    return std::make_unique<FrameSink>(
        device.video, sinks.empty() ? nullptr : compose_metadata_sinks(std::move(sinks)), output.video_output);
}
} // namespace plugins::wrist
