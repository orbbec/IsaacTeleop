// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "metadata_sink.hpp"

#include <mcap/recording_traits.hpp>
#include <mcap/tracker_channels.hpp>
#include <nlohmann/json.hpp>
#include <schema/orbbec_wrist_camera_bfbs_generated.h>
#include <schema/orbbec_wrist_imu_bfbs_generated.h>
#include <schema/timestamp_generated.h>

#include <array>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <utility>

namespace plugins::wrist
{
namespace
{
class CheckedOutput : public mcap::IWritable
{
public:
    explicit CheckedOutput(const std::string& filename)
    {
        file_ = std::fopen(filename.c_str(), "wbx");
        if (!file_)
            throw std::runtime_error("Cannot exclusively create Orbbec Wrist MCAP partial output");
    }
    ~CheckedOutput() override
    {
        if (file_)
            std::fclose(file_);
    }
    void handleWrite(const std::byte* data, uint64_t size) override
    {
        if (!file_ || std::fwrite(data, 1, size, file_) != size)
            throw std::runtime_error("Orbbec Wrist MCAP write failed");
        size_ += size;
    }
    uint64_t size() const override
    {
        return size_;
    }
    void end() override
    {
        if (!file_)
            return;
        const auto flush = std::fflush(file_);
        const auto close = std::fclose(std::exchange(file_, nullptr));
        if (flush != 0 || close != 0)
            throw std::runtime_error("Orbbec Wrist MCAP flush/close failed");
    }

private:
    FILE* file_ = nullptr;
    uint64_t size_ = 0;
};

class McapMetadataSink final : public IMetadataSink
{
public:
    McapMetadataSink(const std::string& filename,
                     const CaptureConfig& capture,
                     const DeviceInfo& device,
                     bool embedded_media,
                     const std::string& collection_prefix,
                     const std::string& plugin_root_id)
        : filename_(filename), capture_(capture)
    {
        try
        {
            if (std::filesystem::exists(filename) || std::filesystem::is_symlink(filename))
                throw std::runtime_error("Orbbec Wrist MCAP destination already exists");
            const std::filesystem::path path(filename);
            if (!path.parent_path().empty())
                std::filesystem::create_directories(path.parent_path());
            file_ = std::make_unique<CheckedOutput>(filename + ".partial");
            mcap::McapWriterOptions options("wrist");
            options.compression = mcap::Compression::None;
            writer_.open(*file_, options);
            const auto metadata_prefix = collection_prefix.empty() ? "wrist_camera" : collection_prefix;
            const auto media_prefix =
                collection_prefix.empty() ? "orbbec_wrist_media" : collection_prefix + "/EncodedVideo";
            video_channel_ = std::make_unique<core::McapTrackerChannels<core::OrbbecWristFrameMetadataRecord>>(
                writer_, metadata_prefix,
                std::vector<std::string>{ core::OrbbecWristFrameMetadataRecordingTraits::recording_channels[0] });
            if (embedded_media)
                media_video_channel_ =
                    std::make_unique<core::McapTrackerChannels<core::OrbbecWristEncodedVideoFrameRecord>>(
                        writer_, media_prefix, std::vector<std::string>{ "Color" });
            if (capture.enable_imu)
            {
                const std::vector<std::string> channels = { core::OrbbecWristImuRecordingTraits::recording_channels[0] };
                imu_channels_[0] = std::make_unique<core::McapTrackerChannels<core::OrbbecWristImuBatchRecord>>(
                    writer_, metadata_prefix + "/Accel", channels);
                imu_channels_[1] = std::make_unique<core::McapTrackerChannels<core::OrbbecWristImuBatchRecord>>(
                    writer_, metadata_prefix + "/Gyro", channels);
            }
            mcap::Metadata metadata;
            metadata.name = "wrist.capture";
            metadata.metadata = {
                { "serial_number", device.serial_number },
                { "firmware", device.firmware },
                { "media_mode", embedded_media ? "embedded" : "sidecar" },
                { "timestamp_policy",
                  "CLOCK_MONOTONIC retrieval proxy; raw device/system microseconds; no clock synchronization" },
                { "video_policy",
                  "Wrist FrameSink recording gate; recognized Orbbec timestamp SEI removed; parameterized keyframe after sequence gaps and epochs" }
            };
            if (!collection_prefix.empty())
            {
                metadata.metadata["fragment_version"] = "1";
                metadata.metadata["collection_prefix"] = collection_prefix;
                metadata.metadata["plugin_root_id"] = plugin_root_id;
            }
            const auto status = writer_.write(metadata);
            if (!status.ok())
                throw std::runtime_error(status.message);
            mcap::Metadata calibration;
            calibration.name = "wrist.calibration";
            calibration.metadata = { { "json", device.calibration_json },
                                     { "collection_prefix", collection_prefix },
                                     { "serial_number", device.serial_number } };
            if (const auto result = writer_.write(calibration); !result.ok())
                throw std::runtime_error(result.message);
            mcap::Schema schema("wrist.CaptureStatus", "jsonschema",
                                R"({"type":"object","required":["state","error","observed_monotonic_ns"]})");
            writer_.addSchema(schema);
            mcap::Channel channel(metadata_prefix + "/Diagnostics", "json", schema.id);
            writer_.addChannel(channel);
            diagnostic_channel_ = channel.id;
        }
        catch (...)
        {
            writer_.terminate();
            throw;
        }
    }
    ~McapMetadataSink() override
    {
        try
        {
            close();
        }
        catch (...)
        {
            writer_.terminate();
        }
    }
    void on_frame_metadata(const VideoFrame& frame) override
    {
        record(video_channel_.get(), frame_payload(frame), frame.time);
    }
    void on_encoded_video_frame(const VideoFrame& frame) override
    {
        if (media_video_channel_)
            record(media_video_channel_.get(), encoded_frame_payload(frame), frame.time);
    }
    void on_imu_batch(const CaptureBatch& batch, ImuSensor sensor) override
    {
        if (batch.imu.empty())
            return;
        record(imu_channels_[sensor == ImuSensor::Accel ? 0 : 1].get(), imu_payload(batch, sensor, capture_),
               batch.imu.back().time);
    }
    void on_capture_status(CaptureState state, const std::string& error) override
    {
        if (last_state_ == state && last_error_ == error)
            return;
        if (closed_)
            throw std::runtime_error("Orbbec Wrist MCAP is closed");
        const auto now = monotonic_now_ns();
        const auto json = nlohmann::json{
            { "state", static_cast<int>(state) }, { "error", error }, { "observed_monotonic_ns", now }
        }.dump();
        mcap::Message message;
        message.channelId = diagnostic_channel_;
        message.logTime = now;
        message.publishTime = now;
        message.data = reinterpret_cast<const std::byte*>(json.data());
        message.dataSize = json.size();
        if (const auto result = writer_.write(message); !result.ok())
            throw std::runtime_error(result.message);
        last_state_ = state;
        last_error_ = error;
    }
    void close() override
    {
        if (closed_)
        {
            if (!sealed_)
                throw std::runtime_error("Orbbec Wrist MCAP finalization previously failed");
            return;
        }
        closed_ = true;
        try
        {
            writer_.close();
            file_->end();
            sealed_ = true;
        }
        catch (...)
        {
            writer_.terminate();
            throw;
        }
    }
    void commit() override
    {
        if (aborted_ || !sealed_)
            throw std::runtime_error("Cannot commit unsealed or aborted Orbbec Wrist MCAP");
        if (committed_)
            return;
        if (std::filesystem::exists(filename_) || std::filesystem::is_symlink(filename_))
            throw std::runtime_error("Orbbec Wrist MCAP final output appeared before promotion");
        // An exclusive hard link prevents rename's overwrite race with another capture process.
        std::filesystem::create_hard_link(filename_ + ".partial", filename_);
        committed_ = true;
        std::filesystem::remove(filename_ + ".partial");
    }
    void abort() override
    {
        aborted_ = true;
        close();
    }

private:
    template <class Record>
    void record(core::McapTrackerChannels<Record>* channel,
                const core::record_payload_t<Record>& native,
                const FrameTime& time,
                size_t channel_index = 0)
    {
        if (closed_)
            throw std::runtime_error("Orbbec Wrist MCAP is closed");
        if (!channel)
            return;
        const auto count = writer_.statistics().messageCount;
        channel->write(
            channel_index,
            core::pack_record<Record>(&native, core::DeviceDataTimestamp(monotonic_now_ns(), time.retrieved_monotonic_ns,
                                                                         device_timestamp_ns(time.device_us))));
        if (writer_.statistics().messageCount != count + 1)
            throw std::runtime_error("Orbbec Wrist MCAP record was rejected");
    }
    std::string filename_;
    CaptureConfig capture_;
    std::unique_ptr<CheckedOutput> file_;
    mcap::McapWriter writer_;
    std::unique_ptr<core::McapTrackerChannels<core::OrbbecWristFrameMetadataRecord>> video_channel_;
    std::unique_ptr<core::McapTrackerChannels<core::OrbbecWristEncodedVideoFrameRecord>> media_video_channel_;
    std::array<std::unique_ptr<core::McapTrackerChannels<core::OrbbecWristImuBatchRecord>>, 2> imu_channels_;
    uint16_t diagnostic_channel_ = 0;
    std::optional<CaptureState> last_state_;
    std::string last_error_;
    bool closed_ = false;
    bool sealed_ = false;
    bool committed_ = false;
    bool aborted_ = false;
};
} // namespace

std::unique_ptr<IMetadataSink> make_mcap_metadata_sink(const std::string& filename,
                                                       const CaptureConfig& capture,
                                                       const DeviceInfo& device,
                                                       bool embedded_media,
                                                       const std::string& collection_prefix,
                                                       const std::string& plugin_root_id)
{
    return std::make_unique<McapMetadataSink>(
        filename, capture, device, embedded_media, collection_prefix, plugin_root_id);
}
} // namespace plugins::wrist
