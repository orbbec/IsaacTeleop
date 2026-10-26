// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "backend.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <time.h>
#include <utility>

namespace plugins::wrist
{

CaptureError::CaptureError(ErrorCode code, const std::string& message, int vendor_status, int vendor_exception)
    : std::runtime_error(message), code_(code), vendor_status_(vendor_status), vendor_exception_(vendor_exception)
{
}

int64_t monotonic_now_ns()
{
    timespec time{};
    if (clock_gettime(CLOCK_MONOTONIC, &time) != 0)
        throw CaptureError(ErrorCode::VendorError, "CLOCK_MONOTONIC unavailable");
    return static_cast<int64_t>(time.tv_sec) * 1000000000LL + time.tv_nsec;
}

int64_t device_timestamp_ns(uint64_t microseconds)
{
    if (microseconds > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) / 1000)
        throw CaptureError(ErrorCode::MalformedFrame, "Device timestamp exceeds int64 nanoseconds");
    return static_cast<int64_t>(microseconds) * 1000;
}

void validate_config(const CaptureConfig& config)
{
    if (config.startup_timeout_ms == 0 || config.stall_timeout_ms == 0 || config.reconnect_interval_ms == 0 ||
        config.video.width != 1600 || config.video.height != 1200 || config.video.fps > INT32_MAX)
        throw CaptureError(ErrorCode::InvalidConfig, "Orbbec Wrist requires 1600x1200 video and valid FPS/timeouts");
    switch (config.video.format)
    {
    case PixelFormat::Mjpg:
    case PixelFormat::H264:
    case PixelFormat::H265:
        break;
    default:
        throw CaptureError(ErrorCode::InvalidConfig, "Only MJPG, H264 and H265 are supported");
    }
    if (config.enable_imu &&
        ((config.imu_rate_hz != 400 && config.imu_rate_hz != 1000) ||
         (config.accel_full_scale_g != 16 && config.accel_full_scale_g != 24) || config.gyro_full_scale_dps != 2000))
        throw CaptureError(
            ErrorCode::InvalidConfig, "IMU requests: 400/1000 Hz, 16/24 g, 2000 dps; device must advertise them");
}

bool is_compatible_wrist(const DeviceInfo& device)
{
    return device.vid == 0x2bc5 && device.pid == 0x1203 && !device.serial_number.empty();
}

DeviceInfo select_wrist_device(const std::vector<DeviceInfo>& devices, const std::string& serial_number)
{
    std::vector<DeviceInfo> candidates;
    for (const auto& device : devices)
        if (is_compatible_wrist(device) && (serial_number.empty() || serial_number == device.serial_number))
            candidates.push_back(device);
    if (candidates.empty())
        throw CaptureError(
            ErrorCode::DeviceNotFound, "No compatible Orbbec Wrist device matches the requested serial number");
    if (candidates.size() != 1)
    {
        std::string message = "Multiple compatible Orbbec Wrist devices; specify --serial-number:";
        for (const auto& device : candidates)
            message += " " + device.serial_number;
        throw CaptureError(ErrorCode::InvalidConfig, message);
    }
    return candidates.front();
}

class WristCamera::Impl
{
public:
    explicit Impl(std::unique_ptr<CaptureBackend> backend) : backend_(std::move(backend))
    {
        if (!backend_)
            throw CaptureError(ErrorCode::InvalidConfig, "Capture backend is required");
    }

    void cancelled() const
    {
        if (config_.stop_requested && config_.stop_requested())
            throw CaptureError(ErrorCode::Cancelled, "Orbbec Wrist capture cancelled");
    }

    DeviceInfo open(const CaptureConfig& config)
    {
        if (state_ != CaptureState::Closed)
            throw CaptureError(ErrorCode::InvalidState, "Close Orbbec Wrist before opening it again");
        validate_config(config);
        config_ = config;
        try
        {
            cancelled();
            config_.serial_number = select_wrist_device(backend_->enumerate(), config_.serial_number).serial_number;
            info_ = backend_->open(config_);
            validate_identity(info_);
            state_ = CaptureState::Configured;
            error_.clear();
            return info_;
        }
        catch (...)
        {
            const auto original = std::current_exception();
            cleanup();
            state_ = cleanup_failed_ ? CaptureState::Faulted : CaptureState::Closed;
            std::rethrow_exception(original);
        }
    }

    void validate_identity(const DeviceInfo& info) const
    {
        if (!is_compatible_wrist(info) || info.serial_number != config_.serial_number)
            throw CaptureError(ErrorCode::DeviceNotFound, "Backend opened a different device");
        const auto& request = config_.video;
        if (info.video.width == 0 || info.video.height == 0 || info.video.fps == 0 ||
            info.video.format != request.format || (request.width && request.width != info.video.width) ||
            (request.height && request.height != info.video.height) || (request.fps && request.fps != info.video.fps))
            throw CaptureError(ErrorCode::UnsupportedProfile, "Resolved Color profile differs from request");
    }

    void ready(bool need_video, int64_t hard_deadline)
    {
        const int64_t deadline =
            std::min(hard_deadline, monotonic_now_ns() + static_cast<int64_t>(config_.startup_timeout_ms) * 1000000);
        bool video = !need_video, accel = !config_.enable_imu, gyro = !config_.enable_imu;
        do
        {
            cancelled();
            if (monotonic_now_ns() >= deadline)
                break;
            auto batch = backend_->poll();
            validate_batch(batch);
            if (monotonic_now_ns() >= deadline)
                break;
            video = video || !batch.video.empty();
            for (const auto& sample : batch.imu)
            {
                accel = accel || sample.sensor == ImuSensor::Accel;
                gyro = gyro || sample.sensor == ImuSensor::Gyro;
            }
            if (video && accel && gyro)
                return;
        } while (monotonic_now_ns() < deadline);
        throw CaptureError(ErrorCode::Timeout, "Requested Orbbec Wrist streams did not become ready");
    }

    void start_streams(int64_t hard_deadline = std::numeric_limits<int64_t>::max())
    {
        state_ = CaptureState::Starting;
        cancelled();
        if (monotonic_now_ns() >= hard_deadline)
            throw CaptureError(ErrorCode::Timeout, "Orbbec Wrist recovery deadline expired before stream start");
        if (config_.enable_imu)
        {
            backend_->start_imu();
            ready(false, hard_deadline);
        }
        backend_->start_video();
        ready(true, hard_deadline);
        if (epoch_ == UINT32_MAX)
            throw CaptureError(ErrorCode::InvalidState, "Capture epoch overflow");
        ++epoch_;
        history_ = {};
        last_seen_.fill(monotonic_now_ns());
        state_ = CaptureState::Streaming;
    }

    void start()
    {
        if (state_ == CaptureState::Streaming)
            return;
        if (state_ != CaptureState::Configured)
            throw CaptureError(ErrorCode::InvalidState, "Orbbec Wrist must be configured before start");
        try
        {
            cancelled();
            start_streams();
        }
        catch (const std::exception& error)
        {
            error_ = error.what();
            cleanup();
            state_ = CaptureState::Faulted;
            throw;
        }
    }

    void validate_batch(const CaptureBatch& batch) const
    {
        if (batch.video.size() > 8 || batch.imu.size() > 512)
            throw CaptureError(ErrorCode::MalformedFrame, "Orbbec Wrist batch exceeds bounded capacity");
        size_t bytes = 0;
        for (const auto& frame : batch.video)
        {
            bytes += frame.encoded_data.size();
            if (frame.profile != info_.video || frame.encoded_data.empty() ||
                frame.encoded_data.size() > 32 * 1024 * 1024 || bytes > 64 * 1024 * 1024 ||
                frame.time.retrieved_monotonic_ns <= 0)
                throw CaptureError(ErrorCode::MalformedFrame, "Color frame outside selected profile or payload limit");
            device_timestamp_ns(frame.time.device_us);
        }
        for (const auto& sample : batch.imu)
        {
            if (!config_.enable_imu || (sample.sensor != ImuSensor::Accel && sample.sensor != ImuSensor::Gyro) ||
                sample.time.retrieved_monotonic_ns <= 0 || !std::isfinite(sample.temperature_c) ||
                !std::all_of(sample.value_si.begin(), sample.value_si.end(), [](double v) { return std::isfinite(v); }))
                throw CaptureError(ErrorCode::MalformedFrame, "Invalid IMU sample");
            device_timestamp_ns(sample.time.device_us);
        }
    }

    struct History
    {
        uint64_t sequence;
        uint64_t device_us;
    };
    template <class Frame>
    void annotate(Frame& frame, size_t stream)
    {
        const auto& previous = history_[stream];
        frame.discontinuity =
            previous && (frame.sequence <= previous->sequence || frame.sequence - previous->sequence != 1 ||
                         frame.time.device_us <= previous->device_us);
        history_[stream] = History{ frame.sequence, frame.time.device_us };
        frame.capture_epoch = epoch_;
        last_seen_[stream] = monotonic_now_ns();
    }

    CaptureBatch update()
    {
        cancelled();
        if (state_ == CaptureState::Recovering)
        {
            recover();
            return {};
        }
        if (state_ != CaptureState::Streaming)
            throw CaptureError(ErrorCode::InvalidState, "Orbbec Wrist is not streaming");
        try
        {
            auto batch = backend_->poll();
            validate_batch(batch);
            for (auto& frame : batch.video)
                annotate(frame, 0);
            for (auto& sample : batch.imu)
                annotate(sample, sample.sensor == ImuSensor::Accel ? 1 : 2);
            const auto now = monotonic_now_ns();
            const size_t count = config_.enable_imu ? 3 : 1;
            for (size_t i = 0; i < count; ++i)
                if (now - last_seen_[i] > static_cast<int64_t>(config_.stall_timeout_ms) * 1000000)
                    throw CaptureError(
                        ErrorCode::StreamStalled, "A requested Orbbec Wrist stream stopped producing samples");
            return batch;
        }
        catch (const CaptureError& error)
        {
            error_ = error.what();
            cleanup();
            if (!cleanup_failed_ && config_.reconnect_timeout_ms &&
                (error.code() == ErrorCode::Disconnected || error.code() == ErrorCode::StreamStalled))
            {
                state_ = CaptureState::Recovering;
                recovery_deadline_ = monotonic_now_ns() + static_cast<int64_t>(config_.reconnect_timeout_ms) * 1000000;
                next_retry_ = monotonic_now_ns();
                return {};
            }
            state_ = CaptureState::Faulted;
            throw;
        }
        catch (const std::exception& error)
        {
            error_ = error.what();
            cleanup();
            state_ = CaptureState::Faulted;
            throw;
        }
    }

    void recover()
    {
        const auto now = monotonic_now_ns();
        if (now >= recovery_deadline_)
        {
            state_ = CaptureState::Faulted;
            throw CaptureError(ErrorCode::Timeout, "Orbbec Wrist reconnect deadline expired: " + error_);
        }
        if (now < next_retry_)
            return;
        next_retry_ = now + static_cast<int64_t>(config_.reconnect_interval_ms) * 1000000;
        try
        {
            const auto replacement = backend_->open(config_);
            validate_identity(replacement);
            if (replacement.vid != info_.vid || replacement.pid != info_.pid ||
                replacement.firmware != info_.firmware || replacement.video != info_.video)
                throw CaptureError(
                    ErrorCode::UnsupportedProfile, "Orbbec Wrist identity, firmware or profile changed on reconnect");
            start_streams(recovery_deadline_);
            error_.clear();
        }
        catch (const CaptureError& error)
        {
            error_ = error.what();
            cleanup();
            if (!cleanup_failed_ && (error.code() == ErrorCode::DeviceNotFound || error.code() == ErrorCode::Disconnected ||
                                     error.code() == ErrorCode::ResourceBusy || error.code() == ErrorCode::Timeout))
            {
                state_ = CaptureState::Recovering;
                return;
            }
            state_ = CaptureState::Faulted;
            throw;
        }
        catch (...)
        {
            cleanup();
            state_ = CaptureState::Faulted;
            throw;
        }
    }

    void cleanup() noexcept
    {
        cleanup_failed_ = false;
        try
        {
            backend_->close();
        }
        catch (const std::exception& error)
        {
            cleanup_failed_ = true;
            try
            {
                error_ += std::string("; cleanup: ") + error.what();
            }
            catch (...)
            {
            }
        }
        catch (...)
        {
            cleanup_failed_ = true;
        }
    }

    void stop()
    {
        if (state_ == CaptureState::Faulted)
            return;
        if (state_ == CaptureState::Closed || state_ == CaptureState::Configured)
            return;
        if (state_ == CaptureState::Recovering)
        {
            state_ = CaptureState::Closed;
            return;
        }
        try
        {
            backend_->stop();
            state_ = CaptureState::Configured;
        }
        catch (const std::exception& error)
        {
            error_ = error.what();
            cleanup();
            state_ = CaptureState::Faulted;
            throw;
        }
    }
    void close()
    {
        if (state_ == CaptureState::Closed)
            return;
        cleanup();
        state_ = CaptureState::Closed;
        if (cleanup_failed_)
            throw CaptureError(ErrorCode::CleanupFailed, error_);
    }

    std::unique_ptr<CaptureBackend> backend_;
    CaptureConfig config_;
    DeviceInfo info_;
    CaptureState state_ = CaptureState::Closed;
    std::string error_;
    uint32_t epoch_ = 0;
    std::array<std::optional<History>, 3> history_{};
    std::array<int64_t, 3> last_seen_{};
    int64_t recovery_deadline_ = 0;
    int64_t next_retry_ = 0;
    bool cleanup_failed_ = false;
};

WristCamera::WristCamera(std::unique_ptr<CaptureBackend> backend) : impl_(std::make_unique<Impl>(std::move(backend)))
{
}
WristCamera::~WristCamera()
{
    try
    {
        impl_->close();
    }
    catch (...)
    {
    }
}
DeviceInfo WristCamera::open(const CaptureConfig& config)
{
    return impl_->open(config);
}
void WristCamera::start()
{
    impl_->start();
}
CaptureBatch WristCamera::update()
{
    return impl_->update();
}
void WristCamera::stop()
{
    impl_->stop();
}
void WristCamera::close()
{
    impl_->close();
}
CaptureState WristCamera::state() const
{
    return impl_->state_;
}
const DeviceInfo& WristCamera::device_info() const
{
    return impl_->info_;
}
const std::string& WristCamera::last_error() const
{
    return impl_->error_;
}

} // namespace plugins::wrist
