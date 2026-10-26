// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "core/backend.hpp"
#include "core/metadata_sink.hpp"
#ifdef ORBBEC_WRIST_ENABLE_PREVIEW
#    include "core/preview.hpp"
#endif

#include <log_bridge/logger.hpp>

#include <charconv>
#include <chrono>
#include <csignal>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t stopping = 0;
void on_signal(int)
{
    stopping = 1;
}
uint32_t number(const std::string& value)
{
    uint32_t result = 0;
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), result);
    if (error != std::errc{} || end != value.data() + value.size() || value.empty())
        throw std::invalid_argument("Expected unsigned integer: " + value);
    return result;
}
void usage()
{
    std::cout << "Orbbec Wrist camera plugin\n"
                 "  --list-devices\n"
                 "  [--serial-number=SN] [--collection-prefix=PREFIX] [--mcap-filename=PATH] [--video-output=PATH]\n"
                 "  --format=h264|h265|mjpg --fps=N (default: 30; fixed 1600x1200)\n"
                 "  --media-mode=sidecar|embedded (default: sidecar)\n"
                 "  --mcap-media-spool=PATH (embedded fragment for TeleopSession merger)\n"
                 "  --calibration-output=PATH.json --capabilities-output=PATH.txt --list-capabilities\n"
                 "  One compatible Orbbec Wrist is selected automatically; multiple devices require --serial-number.\n"
                 "  --imu-rate=400|1000 --accel-full-scale=16|24 --disable-imu\n"
                 "  --video-output=PATH (optional encoded Color sidecar)\n"
                 "  --preview (live Color window; Esc/Q or window close stops recording)\n"
                 "  --startup-timeout-ms=N --stall-timeout-ms=N\n"
                 "  --reconnect-timeout-ms=N (0: disabled) --reconnect-interval-ms=N\n"
                 "  --duration=N (seconds; 0: until signal) --plugin-root-id=ID\n"
                 "Profiles are validated against the selected device. No clock synchronization is enabled.\n";
}
} // namespace

int main(int argc, char** argv)
try
{
    using namespace plugins::wrist;
    CaptureConfig capture;
    OutputConfig output;
    bool list = false, list_capabilities = false;
    std::string calibration_output, capabilities_output;
    bool preview_requested = false;
    uint32_t duration = 0;
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (argument == "--help" || argument == "-h")
        {
            usage();
            return 0;
        }
        if (argument == "--list-devices")
        {
            list = true;
            continue;
        }
        if (argument == "--list-capabilities")
        {
            list_capabilities = true;
            continue;
        }
        if (argument == "--disable-imu")
        {
            capture.enable_imu = false;
            continue;
        }
        if (argument == "--preview")
        {
            preview_requested = true;
            continue;
        }
        const auto split = argument.find('=');
        if (split == std::string::npos)
            throw std::invalid_argument("Expected --option=value: " + argument);
        const auto key = argument.substr(0, split), value = argument.substr(split + 1);
        if (key == "--serial-number")
            capture.serial_number = value;
        else if (key == "--collection-prefix")
            output.collection_prefix = value;
        else if (key == "--mcap-filename")
            output.mcap_filename = value;
        else if (key == "--mcap-media-spool")
            output.mcap_media_spool = value;
        else if (key == "--plugin-root-id")
            output.plugin_root_id = value;
        else if (key == "--video-output")
            output.video_output = value;
        else if (key == "--calibration-output")
            calibration_output = value;
        else if (key == "--capabilities-output")
            capabilities_output = value;
        else if (key == "--media-mode")
        {
            if (value != "sidecar" && value != "embedded")
                throw std::invalid_argument("Expected sidecar or embedded media mode");
            output.embedded_media = value == "embedded";
        }
        else if (key == "--fps")
            capture.video.fps = number(value);
        else if (key == "--imu-rate")
            capture.imu_rate_hz = number(value);
        else if (key == "--accel-full-scale")
            capture.accel_full_scale_g = number(value);
        else if (key == "--duration")
            duration = number(value);
        else if (key == "--startup-timeout-ms")
            capture.startup_timeout_ms = number(value);
        else if (key == "--stall-timeout-ms")
            capture.stall_timeout_ms = number(value);
        else if (key == "--reconnect-timeout-ms")
            capture.reconnect_timeout_ms = number(value);
        else if (key == "--reconnect-interval-ms")
            capture.reconnect_interval_ms = number(value);
        else if (key == "--format")
        {
            if (value == "mjpg" || value == "mjpeg")
                capture.video.format = PixelFormat::Mjpg;
            else if (value == "h264")
                capture.video.format = PixelFormat::H264;
            else if (value == "h265")
                capture.video.format = PixelFormat::H265;
            else
                throw std::invalid_argument("Expected h264, h265 or mjpg");
        }
        else
            throw std::invalid_argument("Unknown option: " + key);
    }
    if (list)
    {
        const auto backend = make_orbbec_backend();
        for (const auto& device : backend->enumerate())
            std::cout << device.name << " serial=" << device.serial_number << " uid=" << device.uid << '\n';
        return 0;
    }
    validate_config(capture);
    if (!list_capabilities)
        validate_output_config(output);
    if (!list_capabilities && output.collection_prefix.empty() && output.mcap_filename.empty() &&
        output.video_output.empty())
        throw std::invalid_argument("Specify --collection-prefix, --mcap-filename and/or --video-output");
#ifdef ORBBEC_WRIST_ENABLE_PREVIEW
    std::unique_ptr<Preview> preview;
    if (preview_requested)
        preview = std::make_unique<Preview>();
#else
    if (preview_requested)
        throw std::invalid_argument(
            "--preview requires a build with ORBBEC_WRIST_ENABLE_PREVIEW=ON (orbbec_wrist.sh build --preview)");
#endif
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    capture.stop_requested = [] { return stopping != 0; };
    WristCamera camera(make_orbbec_backend());
    const auto device = camera.open(capture);
    const auto logger = isaaccapture::Logger::get("isaaccapture.plugins.orbbec_wrist.Capture");
    logger->info("Orbbec Wrist selected: {} serial={} firmware={} Color={}x{}@{} IMU={}Hz enabled={}", device.name,
                 device.serial_number, device.firmware, device.video.width, device.video.height, device.video.fps,
                 capture.imu_rate_hz, capture.enable_imu);
    if (list_capabilities)
    {
        std::cout << device.capabilities;
        camera.close();
        return 0;
    }
    const auto save_snapshot = [](const std::string& path, const std::string& content)
    {
        if (path.empty())
            return;
        if (std::filesystem::exists(path) || std::filesystem::is_symlink(path))
            throw std::runtime_error("Snapshot output already exists: " + path);
        const auto parent = std::filesystem::path(path).parent_path();
        if (!parent.empty())
            std::filesystem::create_directories(parent);
        std::ofstream file(path);
        file << content << '\n';
        file.close();
        if (!file)
            throw std::runtime_error("Cannot write capture snapshot: " + path);
    };
    save_snapshot(calibration_output, device.calibration_json);
    save_snapshot(capabilities_output, device.capabilities);
    if (!output.embedded_media && !output.mcap_filename.empty() && output.video_output.empty())
    {
        const auto extension = capture.video.format == PixelFormat::H264 ? "h264" :
                               capture.video.format == PixelFormat::H265 ? "h265" :
                                                                           "mjpg";
        output.video_output =
            (std::filesystem::path(output.mcap_filename).parent_path() / "raw" / (std::string("Color.") + extension))
                .string();
    }
    uint64_t video_frames = 0, imu_samples = 0;
    auto sink = create_frame_sink(output, capture, device);
    const auto status = [&](CaptureState state, const std::string& error)
    {
        if (auto* metadata = sink->metadata_sink())
            metadata->on_capture_status(state, error);
    };
    try
    {
        status(CaptureState::Starting, {});
        camera.start();
        const auto begin = monotonic_now_ns();
        while (!stopping && (!duration || monotonic_now_ns() - begin < static_cast<int64_t>(duration) * 1000000000))
        {
#ifdef ORBBEC_WRIST_ENABLE_PREVIEW
            if (preview)
            {
                preview->update();
                if (preview->closed())
                    break;
            }
#endif
            auto batch = camera.update();
            video_frames += batch.video.size();
            imu_samples += batch.imu.size();
#ifdef ORBBEC_WRIST_ENABLE_PREVIEW
            sink->write(batch,
                        [&](const VideoFrame& frame)
                        {
                            if (preview)
                                preview->submit(frame);
                        });
#else
            sink->write(batch);
#endif
            status(camera.state(), camera.last_error());
            if (camera.state() == CaptureState::Recovering)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        camera.close();
        status(CaptureState::Closed, {});
        sink->close_media();
        sink->close_metadata();
        sink->commit_metadata();
        logger->info("Orbbec Wrist capture finalized: received_video_frames={} received_imu_samples={}", video_frames,
                     imu_samples);
    }
    catch (...)
    {
        const auto original = std::current_exception();
        std::string failure = "Unknown Orbbec Wrist capture failure";
        try
        {
            std::rethrow_exception(original);
        }
        catch (const std::exception& error)
        {
            failure = error.what();
        }
        catch (...)
        {
        }
        try
        {
            status(CaptureState::Faulted, failure);
        }
        catch (...)
        {
        }
        try
        {
            camera.close();
        }
        catch (...)
        {
        }
        try
        {
            sink->close_media();
        }
        catch (...)
        {
        }
        try
        {
            sink->abort_metadata();
        }
        catch (...)
        {
        }
        std::rethrow_exception(original);
    }
    return 0;
}
catch (const plugins::wrist::CaptureError& error)
{
    isaaccapture::Logger::get("isaaccapture.plugins.orbbec_wrist.main")->error("{}", error.what());
    return error.code() == plugins::wrist::ErrorCode::Cancelled ? 130 : 1;
}
catch (const std::exception& error)
{
    isaaccapture::Logger::get("isaaccapture.plugins.orbbec_wrist.main")->error("{}", error.what());
    return 1;
}
