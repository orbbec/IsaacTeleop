// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>
#include <orbbec_camera/orbbec_camera.hpp>

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace
{

class RecordingMetadataSink final : public plugins::orbbec::IMetadataSink
{
public:
    void on_frame_metadata(const plugins::orbbec::CapturedFrame& frame) override
    {
        frames.push_back(frame);
    }
    std::vector<plugins::orbbec::CapturedFrame> frames;
};

class RecordingMediaSink final : public plugins::orbbec::IMetadataSink
{
public:
    void on_frame_metadata(const plugins::orbbec::CapturedFrame&) override
    {
    }
    void on_encoded_video_frame(const plugins::orbbec::CapturedFrame& frame) override
    {
        frames.push_back(frame);
    }
    std::vector<plugins::orbbec::CapturedFrame> frames;
};

void append_nal(std::vector<uint8_t>& bytes, std::initializer_list<uint8_t> header, const std::string& payload = {})
{
    bytes.insert(bytes.end(), { 0, 0, 0, 1 });
    bytes.insert(bytes.end(), header);
    bytes.insert(bytes.end(), payload.begin(), payload.end());
}

std::vector<uint8_t> parameterized_idr(core::OrbbecPixelFormat format)
{
    std::vector<uint8_t> bytes;
    if (format == core::OrbbecPixelFormat_H264)
    {
        append_nal(bytes, { 0x67, 0x01 });
        append_nal(bytes, { 0x68, 0x01 });
        append_nal(bytes, { 0x65, 0x01 });
    }
    else
    {
        append_nal(bytes, { 0x40, 0x01, 0x01 });
        append_nal(bytes, { 0x42, 0x01, 0x01 });
        append_nal(bytes, { 0x44, 0x01, 0x01 });
        append_nal(bytes, { 0x26, 0x01, 0x01 });
    }
    return bytes;
}

void append_sei(std::vector<uint8_t>& bytes, core::OrbbecPixelFormat format, const std::string& payload, bool suffix = false)
{
    if (format == core::OrbbecPixelFormat_H264)
        append_nal(bytes, { 0x06, 0x05, static_cast<uint8_t>(payload.size()) }, payload);
    else
        append_nal(bytes,
                   { static_cast<uint8_t>(suffix ? 0x50 : 0x4e), 0x01, 0x05, static_cast<uint8_t>(payload.size()) },
                   payload);
    bytes.push_back(0x80);
}

std::vector<uint8_t> read_bytes(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary);
    return { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
}

} // namespace

TEST_CASE("Orbbec FrameSink writes raw MJPEG and forwards metadata", "[orbbec][writer][metadata]")
{
    const auto output = std::filesystem::temp_directory_path() / "isaacteleop_orbbec_camera_test.mjpg";
    auto metadata_sink = std::make_unique<RecordingMetadataSink>();
    auto* metadata_sink_ptr = metadata_sink.get();
    {
        plugins::orbbec::FrameSink sink(
            { { core::OrbbecCameraStream_ColorLeft, output.string() } }, std::move(metadata_sink));
        plugins::orbbec::CapturedFrame frame;
        frame.metadata.stream = core::OrbbecCameraStream_ColorLeft;
        frame.metadata.sequence_number = 42;
        frame.metadata.width = 1280;
        frame.metadata.height = 720;
        frame.metadata.fps = 30;
        frame.metadata.pixel_format = core::OrbbecPixelFormat_Mjpg;
        frame.encoded_data = { 0xff, 0xd8, 0x01, 0x02, 0xff, 0xd9 };
        frame.sample_time_local_common_clock_ns = 100;
        frame.sample_time_raw_device_clock_ns = 200;
        sink.on_frame(frame);
        REQUIRE(metadata_sink_ptr->frames.size() == 1);
        REQUIRE(metadata_sink_ptr->frames.front().metadata.sequence_number == 42);
    }

    std::ifstream input(output, std::ios::binary);
    std::vector<char> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    REQUIRE(bytes.size() == 6);
    REQUIRE(static_cast<uint8_t>(bytes.front()) == 0xff);
    REQUIRE(static_cast<uint8_t>(bytes.back()) == 0xd9);
    std::filesystem::remove(output);
}

TEST_CASE("Orbbec FrameSink preserves H264 and H265 elementary stream bytes", "[orbbec][writer]")
{
    for (const auto format : { core::OrbbecPixelFormat_H264, core::OrbbecPixelFormat_H265 })
    {
        const auto suffix = format == core::OrbbecPixelFormat_H264 ? ".h264" : ".h265";
        const auto output = std::filesystem::temp_directory_path() / (std::string("isaacteleop_orbbec") + suffix);
        size_t expected_size = 0;
        {
            plugins::orbbec::StreamConfig stream{ core::OrbbecCameraStream_ColorLeft, output.string() };
            stream.pixel_format = format;
            plugins::orbbec::FrameSink sink({ stream });
            plugins::orbbec::CapturedFrame frame;
            frame.metadata.stream = stream.camera;
            frame.metadata.pixel_format = format;
            if (format == core::OrbbecPixelFormat_H264)
                frame.encoded_data = { 0, 0, 0, 1, 0x67, 0x01, 0, 0, 0, 1, 0x68, 0x01, 0, 0, 0, 1, 0x65, 0x01 };
            else
                frame.encoded_data = { 0, 0, 0, 1, 0x40, 0x01, 0, 0, 0, 1, 0x42, 0x01,
                                       0, 0, 0, 1, 0x44, 0x01, 0, 0, 0, 1, 0x26, 0x01 };
            expected_size = frame.encoded_data.size();
            sink.on_frame(frame);
        }
        std::ifstream input(output, std::ios::binary);
        std::vector<char> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        REQUIRE(bytes.size() == expected_size);
        CHECK(bytes[3] == 1);
        std::filesystem::remove(output);
    }
}

TEST_CASE("Orbbec embedded FrameSink does not require sidecar paths", "[orbbec][writer][mcap]")
{
    auto media_sink = std::make_unique<RecordingMediaSink>();
    auto* media_sink_ptr = media_sink.get();
    plugins::orbbec::StreamConfig stream{ core::OrbbecCameraStream_ColorLeft, "" };
    stream.pixel_format = core::OrbbecPixelFormat_H264;
    plugins::orbbec::FrameSink sink({ stream }, std::move(media_sink), false);
    plugins::orbbec::CapturedFrame frame;
    frame.metadata.stream = stream.camera;
    frame.metadata.pixel_format = stream.pixel_format;
    frame.metadata.sequence_number = 1;
    frame.encoded_data = { 0, 0, 0, 1, 0x67, 1, 0, 0, 0, 1, 0x68, 1, 0, 0, 0, 1, 0x65, 1 };
    sink.on_frame(frame);
    REQUIRE(media_sink_ptr->frames.size() == 1);
    CHECK(media_sink_ptr->frames.front().metadata.encoded_bytes == frame.encoded_data.size());
}

TEST_CASE("Orbbec stream validation delegates encoded FPS support to SDK profile selection", "[orbbec][profile]")
{
    plugins::orbbec::CaptureConfig capture;
    plugins::orbbec::StreamConfig h264{ core::OrbbecCameraStream_ColorLeft, "unused.h264" };
    h264.pixel_format = core::OrbbecPixelFormat_H264;
    h264.fps = 60;
    REQUIRE_NOTHROW(plugins::orbbec::validate_stream_config(h264, capture));

    plugins::orbbec::StreamConfig h265{ core::OrbbecCameraStream_ColorRight, "unused.h265" };
    h265.pixel_format = core::OrbbecPixelFormat_H265;
    capture.fps = 60;
    REQUIRE_NOTHROW(plugins::orbbec::validate_stream_config(h265, capture));

    capture.fps = 0;
    REQUIRE_NOTHROW(plugins::orbbec::validate_stream_config(h265, capture));
}

TEST_CASE("Orbbec resolved-profile certification closes automatic FPS bypass", "[orbbec][profile]")
{
    plugins::orbbec::StreamConfig stream{ core::OrbbecCameraStream_ColorLeft, "unused.h264" };
    stream.pixel_format = core::OrbbecPixelFormat_H264;
    stream.fps = 0;
    CHECK_NOTHROW(plugins::orbbec::validate_resolved_stream_config(stream, 30));
    // This target's Catch2 macro subset omits CHECK_THROWS_WITH, so inspect the exception explicitly.
    bool rejected = false;
    try
    {
        plugins::orbbec::validate_resolved_stream_config(stream, 60);
    }
    catch (const std::invalid_argument& error)
    {
        rejected = true;
        CHECK(std::string(error.what()) ==
              "resolved encoded profile is 60 FPS, but this build certifies encoded recording only through 30 FPS; "
              "no profile fallback was attempted");
    }
    CHECK(rejected);

    stream.pixel_format = core::OrbbecPixelFormat_H265;
    CHECK_THROWS(plugins::orbbec::validate_resolved_stream_config(stream, 60));
    stream.pixel_format = core::OrbbecPixelFormat_Mjpg;
    CHECK_NOTHROW(plugins::orbbec::validate_resolved_stream_config(stream, 60));
}

TEST_CASE("Orbbec FrameSink resumes compressed recording at an IDR after a sequence gap", "[orbbec][writer]")
{
    const auto output = std::filesystem::temp_directory_path() / "isaacteleop_orbbec_gap.h264";
    plugins::orbbec::StreamConfig stream{ core::OrbbecCameraStream_ColorLeft, output.string() };
    stream.pixel_format = core::OrbbecPixelFormat_H264;
    const std::vector<uint8_t> idr = { 0, 0, 0, 1, 0x67, 0x01, 0, 0, 0, 1, 0x68, 0x01, 0, 0, 0, 1, 0x65, 0x01 };
    const std::vector<uint8_t> p_frame = { 0, 0, 0, 1, 0x41, 0x01 };
    {
        plugins::orbbec::FrameSink sink({ stream });
        plugins::orbbec::CapturedFrame frame;
        frame.metadata.stream = stream.camera;
        frame.metadata.pixel_format = stream.pixel_format;
        frame.metadata.sequence_number = 1;
        frame.encoded_data = idr;
        sink.on_frame(frame);

        frame.metadata.sequence_number = 3;
        frame.encoded_data = p_frame;
        sink.on_frame(frame);

        frame.metadata.sequence_number = 4;
        sink.on_frame(frame);

        frame.metadata.sequence_number = 5;
        frame.encoded_data = idr;
        sink.on_frame(frame);
    }
    std::ifstream input(output, std::ios::binary);
    std::vector<char> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    REQUIRE(bytes.size() == idr.size() * 2);
    std::filesystem::remove(output);
}

TEST_CASE("Orbbec FrameSink starts every capture epoch at a parameterized IDR", "[orbbec][writer][reconnect]")
{
    const auto output = std::filesystem::temp_directory_path() / "isaacteleop_orbbec_epoch.h264";
    plugins::orbbec::StreamConfig stream{ core::OrbbecCameraStream_ColorLeft, output.string() };
    stream.pixel_format = core::OrbbecPixelFormat_H264;
    const auto idr = parameterized_idr(stream.pixel_format);
    const std::vector<uint8_t> p_frame = { 0, 0, 0, 1, 0x41, 0x01 };
    {
        plugins::orbbec::FrameSink sink({ stream });
        plugins::orbbec::CapturedFrame frame;
        frame.metadata.stream = stream.camera;
        frame.metadata.pixel_format = stream.pixel_format;
        frame.metadata.sequence_number = 100;
        frame.encoded_data = idr;
        sink.on_frame(frame);
        frame.metadata.sequence_number = 101;
        frame.encoded_data = p_frame;
        sink.on_frame(frame);

        sink.begin_capture_epoch();
        frame.metadata.sequence_number = 0;
        sink.on_frame(frame);
        frame.metadata.sequence_number = 1;
        frame.encoded_data = idr;
        sink.on_frame(frame);
    }
    auto expected = idr;
    expected.insert(expected.end(), p_frame.begin(), p_frame.end());
    expected.insert(expected.end(), idr.begin(), idr.end());
    REQUIRE(read_bytes(output) == expected);
    std::filesystem::remove(output);
}

TEST_CASE("Orbbec FrameSink removes legacy and firmware 1.1.1 timestamp SEI", "[orbbec][writer]")
{
    for (const auto format : { core::OrbbecPixelFormat_H264, core::OrbbecPixelFormat_H265 })
    {
        const auto frame_field = format == core::OrbbecPixelFormat_H264 ? "frameId" : "frame_seq";
        const std::vector<std::string> timestamp_payloads = {
            "ORBBEC,EGO_",
            "EGO0000000000_L,timestamp_us=1788430979164168," + std::string(frame_field) + "=1",
            "EGO0000000000_R,timestamp_us=1788430979180835," + std::string(frame_field) + "=2",
        };
        for (size_t index = 0; index < timestamp_payloads.size(); ++index)
        {
            const auto codec = format == core::OrbbecPixelFormat_H264 ? "h264" : "h265";
            const auto output = std::filesystem::temp_directory_path() /
                                (std::string("isaacteleop_orbbec_timestamp_") + codec + "_" + std::to_string(index));
            const auto expected = parameterized_idr(format);
            auto encoded = expected;
            append_sei(encoded, format, timestamp_payloads[index]);
            if (format == core::OrbbecPixelFormat_H265)
                append_sei(encoded, format, timestamp_payloads[index], true);

            {
                auto media_sink = std::make_unique<RecordingMediaSink>();
                auto* media_sink_ptr = media_sink.get();
                plugins::orbbec::StreamConfig stream{ core::OrbbecCameraStream_ColorLeft, output.string() };
                stream.pixel_format = format;
                plugins::orbbec::FrameSink sink({ stream }, std::move(media_sink));
                plugins::orbbec::CapturedFrame frame;
                frame.metadata.stream = stream.camera;
                frame.metadata.pixel_format = stream.pixel_format;
                frame.metadata.sequence_number = 1;
                frame.encoded_data = std::move(encoded);
                sink.on_frame(frame);
                REQUIRE(media_sink_ptr->frames.size() == 1);
                CHECK(media_sink_ptr->frames.front().encoded_data == expected);
                CHECK(media_sink_ptr->frames.front().metadata.encoded_bytes == expected.size());
            }

            REQUIRE(read_bytes(output) == expected);
            std::filesystem::remove(output);
        }
    }
}

TEST_CASE("Orbbec FrameSink preserves non-timestamp H264 and H265 SEI", "[orbbec][writer]")
{
    for (const auto format : { core::OrbbecPixelFormat_H264, core::OrbbecPixelFormat_H265 })
    {
        const auto frame_field = format == core::OrbbecPixelFormat_H264 ? "frameId" : "frame_seq";
        const std::vector<std::string> ordinary_payloads = {
            "ORBBEC,CAMERA_INFO",
            "EGO0000000000_L,camera_info=stereo",
            "timestamp_us=1788430979164168," + std::string(frame_field) + "=1",
            "EGO0000000000_L," + std::string(frame_field) + "=1,timestamp_us=1788430979164168",
            "EGO0000000000_L,timestamp_us=1788430979164168," +
                std::string(format == core::OrbbecPixelFormat_H264 ? "frame_seq" : "frameId") + "=1",
        };
        const auto codec = format == core::OrbbecPixelFormat_H264 ? "h264" : "h265";
        const auto output =
            std::filesystem::temp_directory_path() / (std::string("isaacteleop_orbbec_ordinary_sei_") + codec);
        auto encoded = parameterized_idr(format);
        for (const auto& payload : ordinary_payloads)
            append_sei(encoded, format, payload);
        if (format == core::OrbbecPixelFormat_H265)
            append_sei(encoded, format, ordinary_payloads.front(), true);

        {
            auto media_sink = std::make_unique<RecordingMediaSink>();
            auto* media_sink_ptr = media_sink.get();
            plugins::orbbec::StreamConfig stream{ core::OrbbecCameraStream_ColorLeft, output.string() };
            stream.pixel_format = format;
            plugins::orbbec::FrameSink sink({ stream }, std::move(media_sink));
            plugins::orbbec::CapturedFrame frame;
            frame.metadata.stream = stream.camera;
            frame.metadata.pixel_format = stream.pixel_format;
            frame.metadata.sequence_number = 1;
            frame.encoded_data = encoded;
            sink.on_frame(frame);
            REQUIRE(media_sink_ptr->frames.size() == 1);
            CHECK(media_sink_ptr->frames.front().encoded_data == encoded);
        }

        REQUIRE(read_bytes(output) == encoded);
        std::filesystem::remove(output);
    }
}

TEST_CASE("Orbbec local MCAP and SchemaPusher modes are mutually exclusive", "[orbbec][cli][mcap]")
{
    plugins::orbbec::CaptureConfig config;
    config.collection_prefix = "ego";
    config.mcap_filename = "metadata.mcap";
    REQUIRE_THROWS_AS(plugins::orbbec::create_frame_sink({}, config), std::invalid_argument);
}

TEST_CASE("Orbbec local MCAP is promoted only by an explicit successful close", "[orbbec][mcap][shutdown]")
{
    const auto base = std::filesystem::temp_directory_path() / "isaacteleop_orbbec_shutdown_test.mcap";
    const auto partial = std::filesystem::path(base.string() + ".partial");
    plugins::orbbec::CaptureConfig config;
    config.mcap_media_mode = plugins::orbbec::McapMediaMode::Embedded;
    plugins::orbbec::StreamConfig stream{ core::OrbbecCameraStream_ColorLeft, "" };
    stream.pixel_format = core::OrbbecPixelFormat_H264;

    std::filesystem::remove(base);
    std::filesystem::remove(partial);
    config.mcap_filename = base.string();
    {
        auto sink = plugins::orbbec::create_frame_sink({ stream }, config);
        sink->abort_metadata();
    }
    CHECK_FALSE(std::filesystem::exists(base));
    CHECK(std::filesystem::exists(partial));
    std::filesystem::remove(partial);

    {
        auto sink = plugins::orbbec::create_frame_sink({ stream }, config);
        sink->close_metadata();
    }
    CHECK(std::filesystem::exists(base));
    CHECK_FALSE(std::filesystem::exists(partial));
    std::filesystem::remove(base);
}
