// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "calibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace plugins::ego
{

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

bool populate_stereo_calibration_from_yaml(const std::string& yaml, core::EgoCalibrationT& calibration)
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
        auto result = std::make_shared<core::EgoCameraIntrinsicsT>();
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
        calibration.left_to_right = std::make_shared<core::EgoExtrinsicsT>();
        calibration.left_to_right->rotation = cameras[1].rotation;
        calibration.left_to_right->translation_mm = cameras[1].translation;
    }
    return true;
}

void populate_video_calibration(const std::shared_ptr<ob::Device>& device,
                                const std::map<core::EgoCameraStream, std::shared_ptr<ob::VideoStreamProfile>>& profiles,
                                core::EgoCalibrationT& calibration)
{
    if (profiles.empty())
        return;
    const auto reference =
        profiles.count(core::EgoCameraStream_ColorLeft) ? core::EgoCameraStream_ColorLeft : profiles.begin()->first;
    const auto sensor_type = [](core::EgoCameraStream stream)
    {
        switch (stream)
        {
        case core::EgoCameraStream_ColorLeft:
            return OB_SENSOR_COLOR_LEFT;
        case core::EgoCameraStream_ColorRight:
            return OB_SENSOR_COLOR_RIGHT;
        case core::EgoCameraStream_ColorSideLeft:
            return OB_SENSOR_COLOR_SIDE_LEFT;
        case core::EgoCameraStream_ColorSideRight:
            return OB_SENSOR_COLOR_SIDE_RIGHT;
        default:
            throw std::invalid_argument("Unknown EGO calibration stream");
        }
    };
    auto calibration_profiles = profiles;
    calibration.video_streams.clear();
    for (const auto& [stream, profile] : profiles)
    {
        auto entry = std::make_shared<core::EgoVideoCalibrationT>();
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
            auto result = std::make_shared<core::EgoCameraIntrinsicsT>();
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
            const auto legacy = stream == core::EgoCameraStream_ColorLeft  ? calibration.color_left :
                                stream == core::EgoCameraStream_ColorRight ? calibration.color_right :
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
        if (entry->stream == core::EgoCameraStream_ColorLeft && !calibration.color_left)
            calibration.color_left = entry->intrinsics;
        if (entry->stream == core::EgoCameraStream_ColorRight && !calibration.color_right)
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
            auto result = std::make_shared<core::EgoExtrinsicsT>();
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

std::string raw_data(const std::shared_ptr<ob::Device>& device, OBPropertyID id)
{
    std::vector<uint8_t> result;
    bool failed = false;
    device->getRawData(id,
                       [&result, &failed](OBDataTranState state, OBDataChunk* chunk)
                       {
                           if (state == DATA_TRAN_STAT_TRANSFERRING && chunk && chunk->data && chunk->size)
                           {
                               if (result.size() < chunk->fullDataSize)
                                   result.resize(chunk->fullDataSize);
                               std::copy(chunk->data, chunk->data + chunk->size, result.begin() + chunk->offset);
                           }
                           else if (state < 0)
                               failed = true;
                       });
    if (failed)
        throw std::runtime_error("Ego raw calibration transfer failed");
    return std::string(result.begin(), result.end());
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
