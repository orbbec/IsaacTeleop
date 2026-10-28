// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "calibration.hpp"

#include "inc/ego_camera/raw_calibration.hpp"

#include <yaml-cpp/yaml.h>

// Separate std headers because clang-format's module category does not match yaml-cpp.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace plugins::ego
{

namespace
{

YAML::Node unique_yaml_field(const YAML::Node& mapping, const std::string& name)
{
    YAML::Node result(YAML::NodeType::Undefined);
    if (!mapping.IsMap())
        return result;
    for (const auto& field : mapping)
    {
        if (!field.first.IsScalar() || field.first.as<std::string>() != name)
            continue;
        if (result.IsDefined())
            return YAML::Node(YAML::NodeType::Undefined);
        result = field.second;
    }
    return result;
}

bool valid_imu_transform(const std::array<std::array<double, 4>, 4>& matrix)
{
    constexpr double tolerance = 1e-4;
    for (size_t column = 0; column < 4; ++column)
    {
        if (std::abs(matrix[3][column] - (column == 3 ? 1.0 : 0.0)) > 1e-6)
            return false;
    }
    for (size_t first = 0; first < 3; ++first)
    {
        for (size_t second = 0; second < 3; ++second)
        {
            double product = 0;
            for (size_t row = 0; row < 3; ++row)
                product += matrix[row][first] * matrix[row][second];
            if (std::abs(product - (first == second ? 1.0 : 0.0)) > tolerance)
                return false;
        }
    }
    const auto& a = matrix[0];
    const auto& b = matrix[1];
    const auto& c = matrix[2];
    const double determinant =
        a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
    return std::abs(determinant - 1.0) <= tolerance;
}

} // namespace

bool populate_imu_calibration_from_yaml(const std::string& yaml, core::OrbbecEgoCalibrationT& calibration)
{
    core::OrbbecEgoExtrinsicsT extrinsics;
    try
    {
        std::string parse_input = yaml;
        if (parse_input.rfind("%YAML:1.0", 0) == 0)
            parse_input.replace(0, 9, "%YAML 1.1");
        const auto documents = YAML::LoadAll(parse_input);
        if (documents.size() != 1)
            return false;
        const auto camera = unique_yaml_field(documents.front(), "cam0");
        const auto transform = unique_yaml_field(camera, "T_cam_imu");
        if (!transform.IsSequence() || transform.size() != 4)
            return false;
        std::array<std::array<double, 4>, 4> matrix{};
        for (size_t row = 0; row < 4; ++row)
        {
            if (!transform[row].IsSequence() || transform[row].size() != 4)
                return false;
            for (size_t column = 0; column < 4; ++column)
            {
                const double value = transform[row][column].as<double>();
                if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
                    return false;
                matrix[row][column] = value;
            }
        }
        if (!valid_imu_transform(matrix))
            return false;
        // EGO SDK d49af169 registers T_cam_imu as accel -> cam0, then declares
        // gyro/accel coincident. Its row-major rotation and mm translation copy directly.
        for (size_t row = 0; row < 3; ++row)
        {
            for (size_t column = 0; column < 3; ++column)
                extrinsics.rotation.push_back(static_cast<float>(matrix[row][column]));
            extrinsics.translation_mm.push_back(static_cast<float>(matrix[row][3]));
        }
    }
    catch (const YAML::Exception&)
    {
        return false;
    }

    auto accel = calibration.accel_to_left;
    auto gyro = calibration.gyro_to_left;
    if (!accel)
        accel = std::make_shared<core::OrbbecEgoExtrinsicsT>(extrinsics);
    if (!gyro)
        gyro = std::make_shared<core::OrbbecEgoExtrinsicsT>(extrinsics);
    calibration.accel_to_left = std::move(accel);
    calibration.gyro_to_left = std::move(gyro);
    return true;
}

bool populate_stereo_video_extrinsics(core::OrbbecEgoCalibrationT& calibration)
{
    const auto& canonical = calibration.left_to_right;
    if (!canonical || canonical->rotation.size() != 9 || canonical->translation_mm.size() != 3)
        return false;
    std::array<std::array<double, 4>, 4> matrix{};
    matrix[3][3] = 1.0;
    for (size_t row = 0; row < 3; ++row)
    {
        for (size_t column = 0; column < 3; ++column)
        {
            const auto value = canonical->rotation[row * 3 + column];
            if (!std::isfinite(value))
                return false;
            matrix[row][column] = value;
        }
        if (!std::isfinite(canonical->translation_mm[row]))
            return false;
        matrix[row][3] = canonical->translation_mm[row];
    }
    if (!valid_imu_transform(matrix))
        return false;

    auto inverse = std::make_shared<core::OrbbecEgoExtrinsicsT>();
    // Per-stream transforms point to the left reference: inverse(R, t) = (R^T, -R^T t).
    for (size_t row = 0; row < 3; ++row)
    {
        double translation = 0;
        for (size_t column = 0; column < 3; ++column)
        {
            inverse->rotation.push_back(static_cast<float>(matrix[column][row]));
            translation -= matrix[column][row] * matrix[column][3];
        }
        if (!std::isfinite(translation) || std::abs(translation) > std::numeric_limits<float>::max())
            return false;
        inverse->translation_mm.push_back(static_cast<float>(translation));
    }
    bool populated = false;
    for (auto& entry : calibration.video_streams)
    {
        if (!entry || entry->stream != core::OrbbecEgoCameraStream_ColorRight ||
            entry->reference_stream != core::OrbbecEgoCameraStream_ColorLeft || entry->to_reference)
            continue;
        entry->to_reference = inverse;
        entry->extrinsics_source = "stereo_calibration_inverse";
        entry->extrinsics_error.clear();
        populated = true;
    }
    return populated;
}

std::vector<float> yaml_numbers(const std::string& text)
{
    std::vector<float> numbers;
    const char* cursor = text.c_str();
    while (*cursor != '\0')
    {
        char* end = nullptr;
        const float value = std::strtof(cursor, &end);
        if (end != cursor)
        {
            numbers.push_back(value);
            cursor = end;
        }
        else
            ++cursor;
    }
    return numbers;
}

bool populate_stereo_calibration_from_yaml(const std::string& yaml, core::OrbbecEgoCalibrationT& calibration)
{
    struct Camera
    {
        uint32_t width = 0;
        uint32_t height = 0;
        float fx = 0;
        float fy = 0;
        float cx = 0;
        float cy = 0;
        std::vector<float> distortion;
        std::vector<float> rotation;
        std::vector<float> translation;
    };
    std::vector<Camera> cameras;
    enum class Section
    {
        kNone,
        kIntrinsics,
        kDistortion,
        kRotation,
    };
    Section section = Section::kNone;
    std::istringstream input(yaml);
    std::string line;
    while (std::getline(input, line))
    {
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos)
            continue;
        std::string key = line.substr(first);
        if (!key.empty() && key.back() == '\r')
            key.pop_back();
        if (key.rfind("- id:", 0) == 0)
        {
            cameras.emplace_back();
            section = Section::kNone;
            continue;
        }
        if (cameras.empty())
            continue;
        auto& camera = cameras.back();
        if (key == "intrinsics:")
        {
            section = Section::kIntrinsics;
            continue;
        }
        if (key == "distortion:")
        {
            section = Section::kDistortion;
            continue;
        }
        if (key == "rotation:")
        {
            section = Section::kRotation;
            continue;
        }
        if (key.rfind("translation:", 0) == 0)
        {
            camera.translation = yaml_numbers(key);
            section = Section::kNone;
            continue;
        }
        if (section == Section::kRotation && key.rfind("- [", 0) == 0)
        {
            const auto values = yaml_numbers(key);
            camera.rotation.insert(camera.rotation.end(), values.begin(), values.end());
            continue;
        }
        const auto colon = key.find(':');
        if (colon == std::string::npos)
            continue;
        const auto values = yaml_numbers(key.substr(colon + 1));
        if (values.empty())
            continue;
        if (key.rfind("image_width:", 0) == 0)
            camera.width = static_cast<uint32_t>(values.front());
        else if (key.rfind("image_height:", 0) == 0)
            camera.height = static_cast<uint32_t>(values.front());
        else if (section == Section::kIntrinsics)
        {
            if (key.rfind("fx:", 0) == 0)
                camera.fx = values.front();
            else if (key.rfind("fy:", 0) == 0)
                camera.fy = values.front();
            else if (key.rfind("cx:", 0) == 0)
                camera.cx = values.front();
            else if (key.rfind("cy:", 0) == 0)
                camera.cy = values.front();
        }
        else if (section == Section::kDistortion && (key.rfind("k", 0) == 0 || key.rfind("p", 0) == 0))
            camera.distortion.push_back(values.front());
    }
    if (cameras.size() < 2 || cameras[0].width == 0 || cameras[1].width == 0 || cameras[0].fx == 0 || cameras[1].fx == 0)
        return false;
    const auto intrinsics = [](const Camera& camera)
    {
        auto result = std::make_shared<core::OrbbecEgoCameraIntrinsicsT>();
        result->width = camera.width;
        result->height = camera.height;
        result->fx = camera.fx;
        result->fy = camera.fy;
        result->cx = camera.cx;
        result->cy = camera.cy;
        result->distortion_model = OB_DISTORTION_KANNALA_BRANDT4;
        result->distortion = camera.distortion;
        return result;
    };
    calibration.color_left = intrinsics(cameras[0]);
    calibration.color_right = intrinsics(cameras[1]);
    if (cameras[1].rotation.size() == 9 && cameras[1].translation.size() == 3)
    {
        calibration.left_to_right = std::make_shared<core::OrbbecEgoExtrinsicsT>();
        calibration.left_to_right->rotation = cameras[1].rotation;
        calibration.left_to_right->translation_mm = cameras[1].translation;
    }
    return true;
}

void populate_video_calibration(const std::shared_ptr<ob::Device>& device,
                                const std::map<core::OrbbecEgoCameraStream, std::shared_ptr<ob::VideoStreamProfile>>& profiles,
                                core::OrbbecEgoCalibrationT& calibration)
{
    if (profiles.empty())
        return;
    const auto reference = profiles.count(core::OrbbecEgoCameraStream_ColorLeft) ? core::OrbbecEgoCameraStream_ColorLeft :
                                                                                   profiles.begin()->first;
    const auto sensor_type = [](core::OrbbecEgoCameraStream stream)
    {
        switch (stream)
        {
        case core::OrbbecEgoCameraStream_ColorLeft:
            return OB_SENSOR_COLOR_LEFT;
        case core::OrbbecEgoCameraStream_ColorRight:
            return OB_SENSOR_COLOR_RIGHT;
        case core::OrbbecEgoCameraStream_ColorSideLeft:
            return OB_SENSOR_COLOR_SIDE_LEFT;
        case core::OrbbecEgoCameraStream_ColorSideRight:
            return OB_SENSOR_COLOR_SIDE_RIGHT;
        default:
            throw std::invalid_argument("Unknown EGO calibration stream");
        }
    };
    auto calibration_profiles = profiles;
    calibration.video_streams.clear();
    for (const auto& [stream, profile] : profiles)
    {
        auto entry = std::make_shared<core::OrbbecEgoVideoCalibrationT>();
        entry->stream = stream;
        entry->reference_stream = reference;
        const auto read_intrinsics = [&entry](const std::shared_ptr<ob::VideoStreamProfile>& candidate)
        {
            const auto source = candidate->getIntrinsic();
            const auto distortion = candidate->getDistortion();
            if (source.width <= 0 || source.height <= 0 || static_cast<uint32_t>(source.width) != candidate->getWidth() ||
                static_cast<uint32_t>(source.height) != candidate->getHeight() || !std::isfinite(source.fx) ||
                !std::isfinite(source.fy) || source.fx <= 0 || source.fy <= 0 || !std::isfinite(source.cx) ||
                !std::isfinite(source.cy))
                throw std::runtime_error("SDK returned invalid camera intrinsics");
            auto result = std::make_shared<core::OrbbecEgoCameraIntrinsicsT>();
            result->width = source.width;
            result->height = source.height;
            result->fx = source.fx;
            result->fy = source.fy;
            result->cx = source.cx;
            result->cy = source.cy;
            result->distortion_model = distortion.model;
            result->distortion = { distortion.k1, distortion.k2, distortion.k3, distortion.k4,
                                   distortion.k5, distortion.k6, distortion.p1, distortion.p2 };
            if (!std::all_of(
                    result->distortion.begin(), result->distortion.end(), [](float v) { return std::isfinite(v); }))
                throw std::runtime_error("SDK returned invalid camera distortion");
            entry->intrinsics = std::move(result);
        };
        try
        {
            read_intrinsics(profile);
            entry->intrinsics_source = "sdk_stream_profile";
        }
        catch (const std::exception& error)
        {
            entry->intrinsics_error = error.what();
            try
            {
                auto fallback = device->getSensor(sensor_type(stream))
                                    ->getStreamProfileList()
                                    ->getVideoStreamProfile(profile->getWidth(), profile->getHeight(), OB_FORMAT_MJPG, 0);
                calibration_profiles[stream] = fallback;
                read_intrinsics(fallback);
                entry->intrinsics_source = "sdk_mjpg_profile";
                entry->intrinsics_error.clear();
            }
            catch (const std::exception& fallback_error)
            {
                entry->intrinsics_error += std::string("; MJPG: ") + fallback_error.what();
            }
        }
        // The legacy stereo YAML has no verified four-camera identity mapping.
        if (!entry->intrinsics)
        {
            const auto legacy = stream == core::OrbbecEgoCameraStream_ColorLeft  ? calibration.color_left :
                                stream == core::OrbbecEgoCameraStream_ColorRight ? calibration.color_right :
                                                                                   nullptr;
            if (legacy)
            {
                entry->intrinsics = legacy;
                entry->intrinsics_source = "legacy_stereo_calibration";
                entry->intrinsics_error.clear();
            }
        }
        if (entry->intrinsics_source.empty())
            entry->intrinsics_source = "unavailable";
        calibration.video_streams.push_back(std::move(entry));
    }
    for (auto& entry : calibration.video_streams)
    {
        if (entry->stream == core::OrbbecEgoCameraStream_ColorLeft && !calibration.color_left)
            calibration.color_left = entry->intrinsics;
        if (entry->stream == core::OrbbecEgoCameraStream_ColorRight && !calibration.color_right)
            calibration.color_right = entry->intrinsics;
        if (entry->stream == reference)
        {
            entry->extrinsics_source = "reference_stream";
            continue;
        }
        try
        {
            const auto source =
                calibration_profiles.at(entry->stream)->getExtrinsicTo(calibration_profiles.at(reference));
            auto result = std::make_shared<core::OrbbecEgoExtrinsicsT>();
            result->rotation.assign(std::begin(source.rot), std::end(source.rot));
            result->translation_mm.assign(std::begin(source.trans), std::end(source.trans));
            for (size_t row = 0; row < 3; ++row)
            {
                float norm = 0;
                for (size_t col = 0; col < 3; ++col)
                    norm += source.rot[row * 3 + col] * source.rot[row * 3 + col];
                if (!std::isfinite(norm) || std::abs(norm - 1.0f) > 0.05f)
                    throw std::runtime_error("SDK returned invalid extrinsic rotation");
                for (size_t other = row + 1; other < 3; ++other)
                {
                    float dot = 0;
                    for (size_t col = 0; col < 3; ++col)
                        dot += source.rot[row * 3 + col] * source.rot[other * 3 + col];
                    if (!std::isfinite(dot) || std::abs(dot) > 0.05f)
                        throw std::runtime_error("SDK returned nonorthogonal extrinsic rotation");
                }
            }
            const auto& r = source.rot;
            const float determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
                                      r[2] * (r[3] * r[7] - r[4] * r[6]);
            if (!std::isfinite(determinant) || std::abs(determinant - 1.0f) > 0.05f)
                throw std::runtime_error("SDK returned improper extrinsic rotation");
            if (!std::all_of(result->translation_mm.begin(), result->translation_mm.end(),
                             [](float v) { return std::isfinite(v); }))
                throw std::runtime_error("SDK returned invalid extrinsic translation");
            entry->to_reference = std::move(result);
            entry->extrinsics_source = "sdk_stream_profile";
        }
        catch (const std::exception& error)
        {
            entry->extrinsics_source = "unavailable";
            entry->extrinsics_error = error.what();
        }
    }
}

void detail::RawCalibrationTransfer::append(size_t total_size, size_t offset, const uint8_t* data, size_t size) noexcept
{
    if (failed_)
        return;
    // The SDK delivers contiguous chunks. Reject gaps and overlaps before copying.
    if (completed_ || !data || !size || !total_size || offset > total_size || size > total_size - offset ||
        offset != data_.size() || (total_size_ && total_size_ != total_size))
    {
        fail();
        return;
    }
    total_size_ = total_size;
    try
    {
        // Allocate only received bytes, not the device's untrusted total length.
        data_.append(reinterpret_cast<const char*>(data), size);
    }
    catch (...)
    {
        exception_ = std::current_exception();
        fail();
    }
}

void detail::RawCalibrationTransfer::complete() noexcept
{
    if (completed_)
        fail();
    completed_ = true;
}

void detail::RawCalibrationTransfer::fail() noexcept
{
    failed_ = true;
}

std::string detail::RawCalibrationTransfer::finish() const
{
    if (exception_)
        std::rethrow_exception(exception_);
    if (failed_ || !completed_ || !total_size_ || data_.size() != total_size_)
        throw std::runtime_error("Ego raw calibration transfer failed");
    return data_;
}

std::string raw_data(const std::shared_ptr<ob::Device>& device, OBPropertyID id)
{
    detail::RawCalibrationTransfer transfer;
    device->getRawData(id,
                       [&transfer](OBDataTranState state, OBDataChunk* chunk) noexcept
                       {
                           if (state == DATA_TRAN_STAT_TRANSFERRING && chunk)
                               transfer.append(chunk->fullDataSize, chunk->offset, chunk->data, chunk->size);
                           // DONE carries no payload, so handle it independently of chunk data.
                           else if (state == DATA_TRAN_STAT_DONE)
                               transfer.complete();
                           else
                               transfer.fail();
                       });
    return transfer.finish();
}

std::string json_escape(const std::string& input)
{
    std::string output;
    output.reserve(input.size());
    for (const char character : input)
    {
        switch (character)
        {
        case '\\':
            output += "\\\\";
            break;
        case '"':
            output += "\\\"";
            break;
        case '\n':
            output += "\\n";
            break;
        case '\r':
            output += "\\r";
            break;
        case '\t':
            output += "\\t";
            break;
        default:
            output += character;
            break;
        }
    }
    return output;
}

} // namespace plugins::ego
