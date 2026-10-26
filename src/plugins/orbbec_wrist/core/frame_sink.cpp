// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "frame_sink.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string_view>
#include <utility>

namespace plugins::wrist
{

class FrameSink::Impl
{
public:
    Impl(const VideoProfile& profile, std::unique_ptr<IMetadataSink> metadata_sink, const std::string& media_output)
        : profile_(profile), metadata_sink_(std::move(metadata_sink)), writer_{ nullptr, profile.format }
    {
        if (profile.format != PixelFormat::Mjpg && profile.format != PixelFormat::H264 &&
            profile.format != PixelFormat::H265)
            throw std::invalid_argument("Unsupported Orbbec Wrist output format");
        if (!media_output.empty())
        {
            const std::filesystem::path path(media_output);
            if (std::filesystem::exists(path) || std::filesystem::is_symlink(path))
                throw std::runtime_error("Orbbec Wrist video output already exists");
            if (!path.parent_path().empty())
                std::filesystem::create_directories(path.parent_path());
            writer_.file = std::make_unique<std::ofstream>(path, std::ios::binary | std::ios::trunc);
            if (!*writer_.file)
                throw std::runtime_error("Cannot open Orbbec Wrist video output");
        }
    }

    void check_open() const
    {
        if (media_closed_ || metadata_closed_ || aborted_)
            throw std::runtime_error("Orbbec Wrist frame sink is closed");
        if (metadata_sink_ && !metadata_sink_->error().empty())
            throw std::runtime_error("Orbbec Wrist metadata sink failed: " + metadata_sink_->error());
    }

    void on_frame(const VideoFrame& frame, const std::function<void(const VideoFrame&)>& observer)
    {
        check_open();
        if (frame.profile != profile_)
            throw CaptureError(ErrorCode::MalformedFrame, "Orbbec Wrist output profile changed");
        if (!epoch_ || *epoch_ != frame.capture_epoch)
        {
            begin_capture_epoch();
            epoch_ = frame.capture_epoch;
        }
        const bool sequence_gap = writer_.has_sequence && (frame.sequence <= writer_.last_sequence_number ||
                                                           frame.sequence - writer_.last_sequence_number != 1);
        writer_.has_sequence = true;
        writer_.last_sequence_number = frame.sequence;
        auto bytes = writer_.remove_orbbec_timestamp_sei(frame.encoded_data);
        if (!writer_.accept(bytes, sequence_gap))
            return;
        if (writer_.file)
        {
            writer_.file->write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
            if (!*writer_.file)
                throw std::runtime_error("Orbbec Wrist video write failed");
        }
        VideoFrame accepted = frame;
        accepted.encoded_data = std::move(bytes);
        if (metadata_sink_)
        {
            metadata_sink_->on_frame_metadata(accepted);
            metadata_sink_->on_encoded_video_frame(accepted);
        }
        if (observer)
            observer(accepted);
    }

    void write(const CaptureBatch& batch, const std::function<void(const VideoFrame&)>& observer)
    {
        check_open();
        for (const auto& frame : batch.video)
            on_frame(frame, observer);
        if (!metadata_sink_)
            return;
        for (size_t i = 0; i < pending_imu_.size(); ++i)
        {
            auto& pending = pending_imu_[i];
            const auto sensor = i == 0 ? ImuSensor::Accel : ImuSensor::Gyro;
            for (const auto& sample : batch.imu)
            {
                if (sample.sensor != ImuSensor::Accel && sample.sensor != ImuSensor::Gyro)
                    throw CaptureError(ErrorCode::MalformedFrame, "Unknown Orbbec Wrist IMU sensor");
                if (sample.sensor != sensor)
                    continue;
                if (!pending.imu.empty() && pending.imu.back().capture_epoch != sample.capture_epoch)
                    flush_imu(i);
                if (pending.imu.empty())
                    pending_since_[i] = monotonic_now_ns();
                pending.imu.push_back(sample);
                if (pending.imu.size() == 32)
                    flush_imu(i);
            }
            if (!pending.imu.empty() && monotonic_now_ns() - pending_since_[i] >= 20000000)
                flush_imu(i);
        }
    }

    void flush_imu(size_t index)
    {
        auto& pending = pending_imu_[index];
        if (pending.imu.empty())
            return;
        if (metadata_sink_)
            metadata_sink_->on_imu_batch(pending, index == 0 ? ImuSensor::Accel : ImuSensor::Gyro);
        pending.imu.clear();
    }

    void begin_capture_epoch()
    {
        check_open();
        flush_imu(0);
        flush_imu(1);
        writer_.begin_capture_epoch();
        epoch_.reset();
    }

    void close_media()
    {
        if (media_failed_)
            throw std::runtime_error("Orbbec Wrist video finalization previously failed");
        if (media_closed_)
            return;
        if (writer_.file)
        {
            writer_.file->flush();
            media_failed_ = !*writer_.file;
            writer_.file->close();
            media_failed_ = media_failed_ || writer_.file->fail();
            if (media_failed_)
                throw std::runtime_error("Orbbec Wrist video finalization failed");
        }
        media_closed_ = true;
    }

    void close_metadata()
    {
        if (metadata_closed_)
        {
            if (!metadata_sealed_)
                throw std::runtime_error("Orbbec Wrist metadata finalization previously failed");
            return;
        }
        metadata_closed_ = true;
        std::string failures;
        const auto attempt = [&](auto&& action)
        {
            try
            {
                action();
            }
            catch (const std::exception& error)
            {
                failures += std::string(error.what()) + "; ";
            }
            catch (...)
            {
                failures += "Unknown Orbbec Wrist metadata finalization failure; ";
            }
        };
        attempt([&] { flush_imu(0); });
        attempt([&] { flush_imu(1); });
        if (metadata_sink_)
            attempt([&] { metadata_sink_->close(); });
        if (!failures.empty())
            throw std::runtime_error(failures);
        metadata_sealed_ = true;
    }

    void commit_metadata()
    {
        if (aborted_ || !media_closed_ || media_failed_ || !metadata_sealed_)
            throw std::runtime_error("Cannot commit Orbbec Wrist output before all outputs close successfully");
        if (metadata_sink_)
            metadata_sink_->commit();
    }

    void abort_metadata()
    {
        aborted_ = true;
        pending_imu_ = {};
        if (metadata_sink_)
            metadata_sink_->abort();
    }

    IMetadataSink* metadata_sink()
    {
        return metadata_sink_.get();
    }

private:
    struct Writer
    {
        std::unique_ptr<std::ofstream> file;
        PixelFormat format;
        bool parameter_sets_ready = false;
        std::array<std::vector<uint8_t>, 3> parameters;
        bool skip_rasl = false;
        bool has_sequence = false;
        uint64_t last_sequence_number = 0;

        static size_t start_code_size(const std::vector<uint8_t>& bytes, size_t offset)
        {
            if (offset + 3 <= bytes.size() && bytes[offset] == 0 && bytes[offset + 1] == 0 && bytes[offset + 2] == 1)
                return 3;
            if (offset + 4 <= bytes.size() && bytes[offset] == 0 && bytes[offset + 1] == 0 && bytes[offset + 2] == 0 &&
                bytes[offset + 3] == 1)
                return 4;
            return 0;
        }

        static bool is_orbbec_timestamp_sei(const std::vector<uint8_t>& bytes,
                                            size_t payload_begin,
                                            size_t payload_end,
                                            PixelFormat format)
        {
            const auto begin = bytes.begin() + static_cast<std::ptrdiff_t>(payload_begin);
            const auto end = bytes.begin() + static_cast<std::ptrdiff_t>(payload_end);
            // BM2V961002P, FW 1.1.8: unfiltered H264/H265 packets use
            // EGO0000000000_L,timestamp_us=... with frameId/frame_seq respectively.
            // The wire identity is independent of the Wrist Color stream name.
            static constexpr std::string_view kDevicePrefix = "EGO";
            static constexpr std::string_view kTimestamp = ",timestamp_us=";
            static constexpr std::string_view kH264Frame = ",frameId=";
            static constexpr std::string_view kH265Frame = ",frame_seq=";
            const auto matches = [&bytes, payload_end](size_t& offset, std::string_view text)
            {
                if (offset + text.size() > payload_end ||
                    !std::equal(text.begin(), text.end(), bytes.begin() + static_cast<std::ptrdiff_t>(offset)))
                {
                    return false;
                }
                offset += text.size();
                return true;
            };
            const auto consume_decimal = [&bytes, payload_end](size_t& offset)
            {
                const size_t first = offset;
                while (offset < payload_end && bytes[offset] >= '0' && bytes[offset] <= '9')
                    ++offset;
                return offset != first;
            };

            for (auto marker = std::search(begin, end, kDevicePrefix.begin(), kDevicePrefix.end()); marker != end;
                 marker = std::search(marker + 1, end, kDevicePrefix.begin(), kDevicePrefix.end()))
            {
                size_t offset = static_cast<size_t>(std::distance(bytes.begin(), marker));
                if (!matches(offset, kDevicePrefix))
                    continue;
                const size_t digits_begin = offset;
                if (!consume_decimal(offset) || offset - digits_begin != 10 || offset + 2 > payload_end ||
                    bytes[offset] != '_' || (bytes[offset + 1] != 'L' && bytes[offset + 1] != 'R'))
                {
                    continue;
                }
                offset += 2;
                if (!matches(offset, kTimestamp) || !consume_decimal(offset) ||
                    !matches(offset, format == PixelFormat::H264 ? kH264Frame : kH265Frame) || !consume_decimal(offset))
                {
                    continue;
                }
                if (offset + 1 == payload_end && bytes[offset] == 0x80)
                    return true;
            }
            return false;
        }

        std::vector<uint8_t> remove_orbbec_timestamp_sei(const std::vector<uint8_t>& bytes) const
        {
            if (format == PixelFormat::Mjpg)
                return bytes;

            std::vector<uint8_t> result;
            size_t current = 0;
            while (current < bytes.size())
            {
                const auto code_size = start_code_size(bytes, current);
                if (code_size == 0)
                    return bytes;
                const size_t nal_begin = current + code_size;
                if (nal_begin >= bytes.size())
                    break;
                size_t next = nal_begin;
                while (next < bytes.size() && start_code_size(bytes, next) == 0)
                    ++next;
                const uint8_t nal_type =
                    format == PixelFormat::H264 ? bytes[nal_begin] & 0x1fU : (bytes[nal_begin] >> 1U) & 0x3fU;
                const bool is_sei = format == PixelFormat::H264 ? nal_type == 6 : nal_type == 39 || nal_type == 40;
                if (!is_sei || !is_orbbec_timestamp_sei(bytes, nal_begin + 1, next, format))
                    result.insert(result.end(), bytes.begin() + static_cast<std::ptrdiff_t>(current),
                                  bytes.begin() + static_cast<std::ptrdiff_t>(next));
                current = next;
            }
            return result;
        }

        bool accept(std::vector<uint8_t>& bytes, bool sequence_gap)
        {
            if (format == PixelFormat::Mjpg)
                return true;

            if (sequence_gap)
            {
                // P frames after a missing access unit reference data that is no
                // longer in this elementary stream. Resume from a parameterized IDR.
                parameter_sets_ready = false;
                parameters = {};
                skip_rasl = false;
            }

            bool keyframe = false;
            bool picture = false;
            bool cra = false, rasl = false, trailing = false;
            std::array<bool, 3> present{};
            for (size_t offset = 0; offset < bytes.size();)
            {
                const auto code_size = start_code_size(bytes, offset);
                if (code_size == 0)
                {
                    ++offset;
                    continue;
                }
                const size_t nal_offset = offset + code_size;
                const size_t header_size = format == PixelFormat::H264 ? 1 : 2;
                if (nal_offset + header_size > bytes.size())
                    break;
                size_t next = nal_offset + header_size;
                while (next < bytes.size() && start_code_size(bytes, next) == 0)
                    ++next;
                int parameter = -1;
                if (format == PixelFormat::H264)
                {
                    const uint8_t nal_type = bytes[nal_offset] & 0x1fU;
                    if (nal_type == 7 || nal_type == 8)
                        parameter = nal_type - 7;
                    keyframe = keyframe || nal_type == 5;
                    picture = picture || (nal_type >= 1 && nal_type <= 5);
                }
                else
                {
                    const uint8_t nal_type = (bytes[nal_offset] >> 1U) & 0x3fU;
                    if (nal_type >= 32 && nal_type <= 34)
                        parameter = nal_type - 32;
                    keyframe = keyframe || nal_type == 19 || nal_type == 20 || nal_type == 21;
                    picture = picture || nal_type <= 31;
                    cra = cra || nal_type == 21;
                    rasl = rasl || nal_type == 8 || nal_type == 9;
                    trailing = trailing ||
                               (nal_type <= 31 && nal_type != 6 && nal_type != 7 && nal_type != 8 && nal_type != 9);
                }
                if (parameter >= 0 && !parameter_sets_ready)
                {
                    parameters[parameter].assign(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
                                                 bytes.begin() + static_cast<std::ptrdiff_t>(next));
                    present[parameter] = true;
                }
                offset = next;
            }
            if (!parameter_sets_ready)
            {
                const size_t count = format == PixelFormat::H264 ? 2 : 3;
                if (!keyframe || !std::all_of(parameters.begin(), parameters.begin() + count,
                                              [](const auto& value) { return !value.empty(); }))
                    return false;
                // Parameter-only packets were withheld: replay their bytes, not just their presence flags.
                if (!std::all_of(present.begin(), present.begin() + count, [](bool value) { return value; }))
                {
                    std::vector<uint8_t> complete;
                    for (size_t i = 0; i < count; ++i)
                        complete.insert(complete.end(), parameters[i].begin(), parameters[i].end());
                    complete.insert(complete.end(), bytes.begin(), bytes.end());
                    bytes = std::move(complete);
                }
                parameters = {};
                parameter_sets_ready = true;
                skip_rasl = cra;
            }
            else if (skip_rasl)
            {
                // CRA leading RASL pictures can reference frames absent from this recording.
                if (rasl)
                    return false;
                if (trailing)
                    skip_rasl = false;
            }
            // Timestamp-only SEI packets are not pictures and do not enter recorded video.
            return parameter_sets_ready && picture;
        }

        void begin_capture_epoch()
        {
            parameter_sets_ready = false;
            parameters = {};
            skip_rasl = false;
            has_sequence = false;
        }
    };

    VideoProfile profile_;
    std::unique_ptr<IMetadataSink> metadata_sink_;
    Writer writer_;
    std::optional<uint32_t> epoch_;
    std::array<CaptureBatch, 2> pending_imu_;
    std::array<int64_t, 2> pending_since_{};
    bool media_closed_ = false;
    bool media_failed_ = false;
    bool metadata_closed_ = false;
    bool metadata_sealed_ = false;
    bool aborted_ = false;
};

FrameSink::FrameSink(const VideoProfile& profile,
                     std::unique_ptr<IMetadataSink> metadata_sink,
                     const std::string& media_output)
    : impl_(std::make_unique<Impl>(profile, std::move(metadata_sink), media_output))
{
}
FrameSink::~FrameSink()
{
    try
    {
        impl_->close_media();
    }
    catch (...)
    {
    }
    try
    {
        impl_->close_metadata();
    }
    catch (...)
    {
    }
}
void FrameSink::on_frame(const VideoFrame& frame, const std::function<void(const VideoFrame&)>& accepted_frame)
{
    impl_->on_frame(frame, accepted_frame);
}
void FrameSink::write(const CaptureBatch& batch, const std::function<void(const VideoFrame&)>& accepted_frame)
{
    impl_->write(batch, accepted_frame);
}
void FrameSink::begin_capture_epoch()
{
    impl_->begin_capture_epoch();
}
IMetadataSink* FrameSink::metadata_sink()
{
    return impl_->metadata_sink();
}
void FrameSink::close_media()
{
    impl_->close_media();
}
void FrameSink::close_metadata()
{
    impl_->close_metadata();
}
void FrameSink::commit_metadata()
{
    impl_->commit_metadata();
}
void FrameSink::abort_metadata()
{
    impl_->abort_metadata();
}
std::string FrameSink::metadata_error() const
{
    const auto* sink = impl_->metadata_sink();
    return sink ? sink->error() : std::string{};
}
} // namespace plugins::wrist
