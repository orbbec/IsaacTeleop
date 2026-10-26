// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "backend.hpp"

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Version.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <utility>

namespace plugins::wrist
{
namespace
{
CaptureError translate(const ob::Error& error)
{
    ErrorCode code = ErrorCode::VendorError;
    switch (error.getStatus())
    {
    case OB_ERROR_NO_DEVICE:
        code = ErrorCode::DeviceNotFound;
        break;
    case OB_ERROR_DEVICE_DISCONNECTED:
        code = ErrorCode::Disconnected;
        break;
    case OB_ERROR_RESOURCE_BUSY:
        code = ErrorCode::ResourceBusy;
        break;
    case OB_ERROR_DEVICE_ACCESS_DENIED:
        code = ErrorCode::AccessDenied;
        break;
    case OB_ERROR_WAIT_TIMEOUT:
        code = ErrorCode::Timeout;
        break;
    case OB_ERROR_UNSUPPORTED_OPERATION:
    case OB_ERROR_NOT_IMPLEMENTED:
    case OB_ERROR_ITEM_NOT_FOUND:
        code = ErrorCode::UnsupportedProfile;
        break;
    default:
        break;
    }
    if (error.getExceptionType() == OB_EXCEPTION_TYPE_CAMERA_DISCONNECTED)
        code = ErrorCode::Disconnected;
    return CaptureError(code, std::string("Orbbec ") + error.getFunction() + ": " + error.what(),
                        static_cast<int>(error.getStatus()), static_cast<int>(error.getExceptionType()));
}

OBFormat to_format(PixelFormat format)
{
    switch (format)
    {
    case PixelFormat::Mjpg:
        return OB_FORMAT_MJPG;
    case PixelFormat::H264:
        return OB_FORMAT_H264;
    case PixelFormat::H265:
        return OB_FORMAT_H265;
    }
    throw CaptureError(ErrorCode::InvalidConfig, "Unsupported Orbbec Wrist format");
}

struct PipelineDeleter
{
    void operator()(ob_pipeline* pipeline) const noexcept
    {
        ob_error* error = nullptr;
        ob_delete_pipeline(pipeline, &error);
        if (error)
            ob_delete_error(error);
    }
};
struct FrameDeleter
{
    void operator()(ob_frame* frame) const noexcept
    {
        ob_error* error = nullptr;
        ob_delete_frame(frame, &error);
        if (error)
            ob_delete_error(error);
    }
};
using Pipeline = std::unique_ptr<ob_pipeline, PipelineDeleter>;
using FrameOwner = std::unique_ptr<ob_frame, FrameDeleter>;

Pipeline make_pipeline(const std::shared_ptr<ob::Device>& device)
{
    ob_error* error = nullptr;
    Pipeline pipeline(ob_create_pipeline_with_device(device->getImpl(), &error));
    ob::Error::handle(&error);
    return pipeline;
}

std::shared_ptr<ob::FrameSet> poll_pipeline(const Pipeline& pipeline)
{
    ob_error* error = nullptr;
    FrameOwner frame(ob_pipeline_wait_for_frameset(pipeline.get(), 1, &error));
    if (error)
    {
        const bool timeout = error->status == OB_ERROR_WAIT_TIMEOUT;
        if (timeout)
        {
            ob_delete_error(error);
            return {};
        }
        ob::Error::handle(&error);
    }
    if (!frame)
        return {};
    // Keep the C reference guarded until the C++ owner allocation succeeds.
    auto result = std::make_shared<ob::FrameSet>(frame.get());
    frame.release();
    return result;
}

FrameTime timestamp(const std::shared_ptr<ob::Frame>& frame, int64_t retrieved_ns)
{
    return { frame->getTimeStampUs(), frame->getSystemTimeStampUs(), retrieved_ns };
}

DeviceInfo describe(const std::shared_ptr<ob::DeviceInfo>& info)
{
    return { info->getSerialNumber(),
             info->getUid(),
             info->getName(),
             info->getFirmwareVersion(),
             info->getVid(),
             info->getPid(),
             {} };
}

std::string raw_calibration(const std::shared_ptr<ob::Device>& device, OBPropertyID property)
{
    std::string bytes;
    size_t total = 0;
    bool done = false, failed = false;
    std::exception_ptr failure;
    device->getRawData(property,
                       [&](OBDataTranState state, OBDataChunk* chunk) noexcept
                       {
                           try
                           {
                               if (state == DATA_TRAN_STAT_TRANSFERRING && chunk && !done && !failed && chunk->data &&
                                   chunk->size && chunk->fullDataSize <= 8 * 1024 * 1024 &&
                                   chunk->offset == bytes.size() && chunk->offset <= chunk->fullDataSize &&
                                   chunk->size <= chunk->fullDataSize - chunk->offset &&
                                   (!total || total == chunk->fullDataSize))
                               {
                                   total = chunk->fullDataSize;
                                   bytes.append(reinterpret_cast<const char*>(chunk->data), chunk->size);
                               }
                               else if (state == DATA_TRAN_STAT_DONE && !done)
                                   done = true;
                               else
                                   failed = true;
                           }
                           catch (...)
                           {
                               failure = std::current_exception();
                               failed = true;
                           }
                       });
    if (failure)
        std::rethrow_exception(failure);
    if (failed || !done || !total || bytes.size() != total)
        throw std::runtime_error("Incomplete raw calibration transfer");
    return bytes;
}

void describe_capture(const std::shared_ptr<ob::Device>& device,
                      const std::shared_ptr<ob::VideoStreamProfile>& video,
                      DeviceInfo& info)
{
    using Json = nlohmann::json;
    Json calibration = { { "version", 1 },
                         { "device_uid", info.uid },
                         { "serial_number", info.serial_number },
                         { "color", nullptr },
                         { "raw_alignment_yaml", "" },
                         { "raw_imu_yaml", "" },
                         { "errors", Json::object() } };
    try
    {
        auto calibrated = video;
        OBCameraIntrinsic intrinsic{};
        OBCameraDistortion distortion{};
        try
        {
            intrinsic = calibrated->getIntrinsic();
            distortion = calibrated->getDistortion();
        }
        catch (const ob::Error&)
        {
            // Some firmware indexes calibration by the matching MJPEG profile.
            calibrated = device->getSensor(OB_SENSOR_COLOR)
                             ->getStreamProfileList()
                             ->getVideoStreamProfile(1600, 1200, OB_FORMAT_MJPG, 0);
            intrinsic = calibrated->getIntrinsic();
            distortion = calibrated->getDistortion();
        }
        const std::vector<float> coefficients{ distortion.k1, distortion.k2, distortion.k3, distortion.k4,
                                               distortion.k5, distortion.k6, distortion.p1, distortion.p2 };
        if (intrinsic.width != 1600 || intrinsic.height != 1200 || !std::isfinite(intrinsic.fx) ||
            !std::isfinite(intrinsic.fy) || !std::isfinite(intrinsic.cx) || !std::isfinite(intrinsic.cy) ||
            intrinsic.fx <= 0 || intrinsic.fy <= 0 ||
            !std::all_of(coefficients.begin(), coefficients.end(), [](float value) { return std::isfinite(value); }))
            throw std::runtime_error("SDK returned invalid Color intrinsics");
        calibration["color"] = { { "width", intrinsic.width },
                                 { "height", intrinsic.height },
                                 { "fx", intrinsic.fx },
                                 { "fy", intrinsic.fy },
                                 { "cx", intrinsic.cx },
                                 { "cy", intrinsic.cy },
                                 { "distortion_model", distortion.model },
                                 { "distortion", coefficients } };
    }
    catch (const std::exception& error)
    {
        calibration["errors"]["color"] = error.what();
    }
    for (const auto& [key, property] : { std::pair{ "raw_alignment_yaml", OB_RAW_DATA_ALIGN_CALIB_YAML },
                                         std::pair{ "raw_imu_yaml", OB_RAW_DATA_IMU_CALIB_YAML } })
    {
        try
        {
            calibration[key] = raw_calibration(device, property);
        }
        catch (const std::exception& error)
        {
            calibration["errors"][key] = error.what();
        }
    }
    info.calibration_json = calibration.dump(2);
    std::ostringstream capabilities;
    capabilities << "Device " << info.name << "\nSerial " << info.serial_number << "\nUID " << info.uid << "\nFirmware "
                 << info.firmware << "\nSDK " << ob::Version::getMajor() << '.' << ob::Version::getMinor() << '.'
                 << ob::Version::getPatch() << "\nSelected Color " << info.video.width << 'x' << info.video.height
                 << '@' << info.video.fps << " format=" << static_cast<int>(video->getFormat()) << "\n";
    for (const auto& [name, type] : { std::pair{ "Color", OB_SENSOR_COLOR }, std::pair{ "Accel", OB_SENSOR_ACCEL },
                                      std::pair{ "Gyro", OB_SENSOR_GYRO } })
    {
        try
        {
            auto profiles = device->getSensor(type)->getStreamProfileList();
            capabilities << "Sensor " << name << "\n";
            for (uint32_t i = 0; i < profiles->getCount(); ++i)
            {
                const auto profile = profiles->getProfile(i);
                capabilities << "  format=" << static_cast<int>(profile->getFormat());
                if (type == OB_SENSOR_COLOR)
                {
                    auto value = profile->as<ob::VideoStreamProfile>();
                    capabilities << " " << value->getWidth() << 'x' << value->getHeight() << '@' << value->getFps();
                }
                else if (type == OB_SENSOR_ACCEL)
                {
                    auto value = profile->as<ob::AccelStreamProfile>();
                    capabilities << " sample_rate_enum=" << value->getSampleRate()
                                 << " range_enum=" << value->getFullScaleRange();
                }
                else
                {
                    auto value = profile->as<ob::GyroStreamProfile>();
                    capabilities << " sample_rate_enum=" << value->getSampleRate()
                                 << " range_enum=" << value->getFullScaleRange();
                }
                capabilities << "\n";
            }
        }
        catch (const std::exception& error)
        {
            capabilities << "Unavailable " << name << ": " << error.what() << "\n";
        }
    }
    info.capabilities = capabilities.str();
}

class OrbbecBackend final : public CaptureBackend
{
public:
    ~OrbbecBackend() override
    {
        try
        {
            close();
        }
        catch (...)
        {
        }
    }

    std::vector<DeviceInfo> enumerate() override
    {
        try
        {
            ensure_context();
            auto devices = context_->queryDeviceList();
            std::vector<DeviceInfo> result;
            for (uint32_t i = 0; i < devices->getCount(); ++i)
            {
                DeviceInfo info;
                info.serial_number = devices->getSerialNumber(i);
                info.uid = devices->getUid(i);
                info.name = devices->getName(i);
                info.vid = devices->getVid(i);
                info.pid = devices->getPid(i);
                if (is_compatible_wrist(info))
                    result.push_back(std::move(info));
            }
            return result;
        }
        catch (const ob::Error& error)
        {
            throw translate(error);
        }
    }

    DeviceInfo open(const CaptureConfig& config) override
    {
        validate_config(config);
        if (device_)
            throw CaptureError(ErrorCode::InvalidState, "Orbbec device already open");
        try
        {
            ensure_context();
            auto devices = context_->queryDeviceList();
            bool found = false;
            for (uint32_t i = 0; i < devices->getCount(); ++i)
                found = found || (config.serial_number == devices->getSerialNumber(i) && devices->getVid(i) == 0x2bc5 &&
                                  devices->getPid(i) == 0x1203);
            if (!found)
                throw CaptureError(ErrorCode::DeviceNotFound, "Selected Orbbec Wrist serial number is absent");
            device_ = devices->getDeviceBySN(config.serial_number.c_str());
            config_ = config;
            // Preserve EGO's profile selection and per-device video/IMU organization; Wrist exposes Color.
            const auto video =
                device_->getSensor(OB_SENSOR_COLOR)
                    ->getStreamProfileList()
                    ->getVideoStreamProfile(static_cast<int>(config.video.width), static_cast<int>(config.video.height),
                                            to_format(config.video.format), static_cast<int>(config.video.fps));
            if (video->getFormat() != to_format(config.video.format))
                throw CaptureError(ErrorCode::UnsupportedProfile, "Resolved Color format differs from request");
            info_ = describe(device_->getDeviceInfo());
            info_.video = { config.video.format, video->getWidth(), video->getHeight(), video->getFps() };
            video_config_ = std::make_shared<ob::Config>();
            video_config_->enableStream(video);
            video_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
            if (config.enable_imu)
            {
                const auto rate = config.imu_rate_hz == 400 ? OB_SAMPLE_RATE_400_HZ : OB_SAMPLE_RATE_1_KHZ;
                const auto scale = config.accel_full_scale_g == 24 ? OB_ACCEL_FS_24g : OB_ACCEL_FS_16g;
                const auto accel =
                    device_->getSensor(OB_SENSOR_ACCEL)->getStreamProfileList()->getAccelStreamProfile(scale, rate);
                const auto gyro =
                    device_->getSensor(OB_SENSOR_GYRO)->getStreamProfileList()->getGyroStreamProfile(OB_GYRO_FS_2000dps, rate);
                if (accel->getSampleRate() != rate || accel->getFullScaleRange() != scale ||
                    gyro->getSampleRate() != rate || gyro->getFullScaleRange() != OB_GYRO_FS_2000dps)
                    throw CaptureError(ErrorCode::UnsupportedProfile, "Resolved IMU profile differs from request");
                imu_config_ = std::make_shared<ob::Config>();
                imu_config_->enableStream(accel);
                imu_config_->enableStream(gyro);
            }
            describe_capture(device_, video, info_);
            return info_;
        }
        catch (const ob::Error& error)
        {
            throw translate(error);
        }
    }

    void start_imu() override
    {
        start(imu_pipeline_, imu_config_);
    }
    void start_video() override
    {
        start(video_pipeline_, video_config_);
    }

    CaptureBatch poll() override
    {
        try
        {
            CaptureBatch result;
            const auto imu_deadline = monotonic_now_ns() + 4000000;
            if (imu_pipeline_)
                for (size_t i = 0; i < 256; ++i)
                {
                    auto set = poll_pipeline(imu_pipeline_);
                    if (!set)
                        break;
                    const auto retrieved = monotonic_now_ns();
                    for (auto type : { OB_FRAME_ACCEL, OB_FRAME_GYRO })
                    {
                        const auto raw = set->getFrame(type);
                        if (!raw)
                            continue;
                        ImuSample sample;
                        sample.sensor = type == OB_FRAME_ACCEL ? ImuSensor::Accel : ImuSensor::Gyro;
                        sample.sequence = raw->getIndex();
                        sample.time = timestamp(raw, retrieved);
                        if (type == OB_FRAME_ACCEL)
                        {
                            auto frame = raw->as<ob::AccelFrame>();
                            auto value = frame->getValue();
                            sample.value_si = { value.x, value.y, value.z };
                            sample.temperature_c = frame->getTemperature();
                        }
                        else
                        {
                            auto frame = raw->as<ob::GyroFrame>();
                            auto value = frame->getValue();
                            sample.value_si = { value.x, value.y, value.z };
                            sample.temperature_c = frame->getTemperature();
                        }
                        result.imu.push_back(std::move(sample));
                    }
                    if (monotonic_now_ns() >= imu_deadline)
                        break;
                }
            if (video_pipeline_)
            {
                auto set = poll_pipeline(video_pipeline_);
                if (set)
                {
                    const auto retrieved = monotonic_now_ns();
                    const auto raw = set->getFrame(OB_FRAME_COLOR);
                    if (raw)
                    {
                        const auto frame = raw->as<ob::VideoFrame>();
                        const auto profile = frame->getStreamProfile()->as<ob::VideoStreamProfile>();
                        if (frame->getFormat() != to_format(config_.video.format) || !profile)
                            throw CaptureError(ErrorCode::MalformedFrame, "Unexpected Color frame format");
                        VideoFrame video;
                        video.profile = { config_.video.format, frame->getWidth(), frame->getHeight(), profile->getFps() };
                        video.sequence = frame->getIndex();
                        video.time = timestamp(raw, retrieved);
                        const auto bytes = frame->getDataSize();
                        if (!frame->getData() || bytes == 0 || bytes > 32 * 1024 * 1024)
                            throw CaptureError(ErrorCode::MalformedFrame, "Invalid encoded Color payload");
                        video.encoded_data.assign(frame->getData(), frame->getData() + bytes);
                        result.video.push_back(std::move(video));
                    }
                }
            }
            return result;
        }
        catch (const ob::Error& error)
        {
            throw translate(error);
        }
    }

    void stop() override
    {
        std::unique_ptr<ob_error, decltype(&ob_delete_error)> first_error(nullptr, ob_delete_error);
        for (auto* pipeline : { &video_pipeline_, &imu_pipeline_ })
        {
            if (!*pipeline)
                continue;
            ob_error* error = nullptr;
            ob_pipeline_stop(pipeline->get(), &error);
            if (error && !first_error)
                first_error.reset(error);
            else if (error)
                ob_delete_error(error);
            pipeline->reset();
        }
        auto* error = first_error.release();
        try
        {
            ob::Error::handle(&error);
        }
        catch (const ob::Error& error)
        {
            throw translate(error);
        }
    }

    void close() override
    {
        std::exception_ptr failure;
        try
        {
            stop();
        }
        catch (...)
        {
            failure = std::current_exception();
        }
        video_config_.reset();
        imu_config_.reset();
        device_.reset();
        context_.reset();
        if (failure)
            std::rethrow_exception(failure);
    }

private:
    void ensure_context()
    {
        if (!context_)
            context_ = std::make_unique<ob::Context>();
    }
    void start(Pipeline& pipeline, const std::shared_ptr<ob::Config>& config)
    {
        if (!device_ || !config || pipeline)
            throw CaptureError(ErrorCode::InvalidState, "Invalid Orbbec stream start");
        try
        {
            pipeline = make_pipeline(device_);
            ob_error* error = nullptr;
            // No application user_data survives stop: C polling avoids undocumented callback quiescence.
            ob_pipeline_start_with_config(pipeline.get(), config->getImpl(), &error);
            ob::Error::handle(&error);
        }
        catch (const ob::Error& error)
        {
            throw translate(error);
        }
    }

    std::unique_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    std::shared_ptr<ob::Config> video_config_, imu_config_;
    Pipeline video_pipeline_, imu_pipeline_;
    CaptureConfig config_;
    DeviceInfo info_;
};
} // namespace

std::unique_ptr<CaptureBackend> make_orbbec_backend()
{
    return std::make_unique<OrbbecBackend>();
}
} // namespace plugins::wrist
