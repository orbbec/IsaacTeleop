// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace plugins::wrist
{

enum class PixelFormat
{
    Mjpg,
    H264,
    H265
};
enum class ImuSensor
{
    Accel,
    Gyro
};
enum class CaptureState
{
    Closed,
    Configured,
    Starting,
    Streaming,
    Recovering,
    Faulted
};
enum class ErrorCode
{
    InvalidState,
    InvalidConfig,
    DeviceNotFound,
    UnsupportedProfile,
    Timeout,
    Disconnected,
    StreamStalled,
    MalformedFrame,
    ResourceBusy,
    AccessDenied,
    CleanupFailed,
    VendorError,
    Cancelled
};

class CaptureError : public std::runtime_error
{
public:
    CaptureError(ErrorCode code, const std::string& message, int vendor_status = 0, int vendor_exception = 0);
    ErrorCode code() const noexcept
    {
        return code_;
    }
    int vendor_status() const noexcept
    {
        return vendor_status_;
    }
    int vendor_exception() const noexcept
    {
        return vendor_exception_;
    }

private:
    ErrorCode code_;
    int vendor_status_;
    int vendor_exception_;
};

struct VideoProfile
{
    PixelFormat format = PixelFormat::H264;
    uint32_t width = 1600;
    uint32_t height = 1200;
    uint32_t fps = 30;
    bool operator==(const VideoProfile&) const = default;
};

struct CaptureConfig
{
    // Empty selects the only compatible Wrist. Recovery stays pinned to its serial.
    std::string serial_number;
    VideoProfile video;
    bool enable_imu = true;
    uint32_t imu_rate_hz = 400;
    uint32_t accel_full_scale_g = 24;
    uint32_t gyro_full_scale_dps = 2000;
    uint32_t startup_timeout_ms = 5000;
    uint32_t stall_timeout_ms = 3000;
    uint32_t reconnect_timeout_ms = 30000;
    uint32_t reconnect_interval_ms = 1000;
    std::function<bool()> stop_requested;
};

struct DeviceInfo
{
    std::string serial_number;
    std::string uid;
    std::string name;
    std::string firmware;
    int vid = 0;
    int pid = 0;
    VideoProfile video;
    std::string capabilities = {};
    std::string calibration_json = {};
};

struct FrameTime
{
    uint64_t device_us = 0;
    uint64_t system_us = 0;
    // CLOCK_MONOTONIC at retrieval, not exposure time or SDK host receipt time.
    int64_t retrieved_monotonic_ns = 0;
};

struct VideoFrame
{
    VideoProfile profile;
    uint64_t sequence = 0;
    FrameTime time;
    uint32_t capture_epoch = 0;
    bool discontinuity = false;
    // Encoded bytes are owned. Raw stride/planes and access-unit layout are not inferred.
    std::vector<uint8_t> encoded_data;
};

struct ImuSample
{
    ImuSensor sensor = ImuSensor::Accel;
    uint64_t sequence = 0;
    FrameTime time;
    uint32_t capture_epoch = 0;
    bool discontinuity = false;
    std::array<double, 3> value_si{};
    double temperature_c = 0;
};

struct CaptureBatch
{
    std::vector<VideoFrame> video;
    std::vector<ImuSample> imu;
};

class CaptureBackend;
int64_t monotonic_now_ns();
int64_t device_timestamp_ns(uint64_t microseconds);
void validate_config(const CaptureConfig& config);
bool is_compatible_wrist(const DeviceInfo& device);
DeviceInfo select_wrist_device(const std::vector<DeviceInfo>& devices, const std::string& serial_number);
std::unique_ptr<CaptureBackend> make_orbbec_backend();

// All operations, including destruction, belong to one caller thread. No user callbacks.
class WristCamera
{
public:
    explicit WristCamera(std::unique_ptr<CaptureBackend> backend);
    ~WristCamera();
    WristCamera(const WristCamera&) = delete;
    WristCamera& operator=(const WristCamera&) = delete;
    DeviceInfo open(const CaptureConfig& config);
    void start();
    CaptureBatch update();
    void stop();
    void close();
    CaptureState state() const;
    const DeviceInfo& device_info() const;
    const std::string& last_error() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace plugins::wrist
