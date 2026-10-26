// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>
#include <orbbec_wrist/core/frame_sink.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <utility>

using namespace plugins::wrist;
namespace
{
const VideoProfile profile{ PixelFormat::H264, 640, 480, 30 };
const std::vector<uint8_t> keyframe{ 0, 0, 0, 1, 0x67, 1, 0, 0, 0, 1, 0x68, 1, 0, 0, 0, 1, 0x65, 1 };
const std::vector<uint8_t> predicted{ 0, 0, 0, 1, 0x41, 1 };
VideoFrame frame(uint64_t sequence, const std::vector<uint8_t>& bytes, uint32_t epoch = 1)
{
    return { profile, sequence, { 1234, 5678, 9000 }, epoch, false, bytes };
}
struct Observations
{
    std::vector<VideoFrame> metadata;
    std::vector<VideoFrame> media;
    std::vector<std::pair<ImuSensor, CaptureBatch>> imu;
    int closed = 0;
    int committed = 0;
    int aborted = 0;
    bool fail_close = false;
};
class MemorySink final : public IMetadataSink
{
public:
    explicit MemorySink(Observations& observations) : observations_(observations)
    {
    }
    void on_frame_metadata(const VideoFrame& value) override
    {
        observations_.metadata.push_back(value);
    }
    void on_encoded_video_frame(const VideoFrame& value) override
    {
        observations_.media.push_back(value);
    }
    void on_imu_batch(const CaptureBatch& batch, ImuSensor sensor) override
    {
        observations_.imu.emplace_back(sensor, batch);
    }
    void close() override
    {
        ++observations_.closed;
        if (observations_.fail_close)
            throw std::runtime_error("injected sink close failure");
    }
    void commit() override
    {
        ++observations_.committed;
    }
    void abort() override
    {
        ++observations_.aborted;
    }

private:
    Observations& observations_;
};
} // namespace

TEST_CASE("wrist frame sink gates video after gaps and capture epochs", "[wrist][unit]")
{
    Observations seen;
    FrameSink sink(profile, std::make_unique<MemorySink>(seen));
    sink.on_frame(frame(0, predicted));
    sink.on_frame(frame(1, keyframe));
    sink.on_frame(frame(3, predicted));
    sink.on_frame(frame(4, predicted));
    sink.on_frame(frame(5, keyframe));
    sink.begin_capture_epoch();
    sink.on_frame(frame(6, predicted));
    sink.on_frame(frame(7, keyframe));
    sink.on_frame(frame(8, predicted, 2));
    sink.on_frame(frame(9, keyframe, 2));
    REQUIRE(seen.metadata.size() == 4);
    REQUIRE(seen.media.size() == 4);
    REQUIRE(seen.metadata[0].sequence == 1);
    REQUIRE(seen.metadata[1].sequence == 5);
    REQUIRE(seen.metadata[2].sequence == 7);
    REQUIRE(seen.metadata[3].sequence == 9);
    REQUIRE(seen.media[3].capture_epoch == 2);
    REQUIRE(seen.metadata[0].time.device_us == 1234);
    REQUIRE(seen.metadata[0].time.retrieved_monotonic_ns == 9000);
    sink.close_media();
    sink.close_metadata();
    REQUIRE_THROWS(sink.on_frame(frame(10, keyframe)));
}

TEST_CASE("wrist frame sink shares canonical Orbbec video bytes with every consumer", "[wrist][unit]")
{
    Observations seen;
    FrameSink sink(profile, std::make_unique<MemorySink>(seen));
    std::vector<uint8_t> timestamp_sei{ 0, 0, 0, 1, 6, 5, 55 };
    const std::string marker = "EGO0000000000_L,timestamp_us=1792739705889228,frameId=1";
    timestamp_sei.insert(timestamp_sei.end(), marker.begin(), marker.end());
    timestamp_sei.push_back(0x80);
    auto bytes = keyframe;
    bytes.insert(bytes.end(), timestamp_sei.begin(), timestamp_sei.end());
    std::vector<uint8_t> observed;
    sink.on_frame(frame(0, bytes), [&](const auto& accepted) { observed = accepted.encoded_data; });
    REQUIRE(observed == keyframe);
    REQUIRE(seen.metadata[0].encoded_data == observed);
    REQUIRE(seen.media[0].encoded_data == observed);
    sink.on_frame(frame(1, timestamp_sei));
    REQUIRE(seen.metadata.size() == 1);
    const std::vector<uint8_t> other_sei{ 0, 0, 0, 1, 6, 0x23, 0x80 };
    bytes = predicted;
    bytes.insert(bytes.end(), other_sei.begin(), other_sei.end());
    sink.on_frame(frame(2, bytes));
    REQUIRE(seen.media.back().encoded_data == bytes);
}

TEST_CASE("wrist timestamp SEI matches sampled H264 and H265 wire identities", "[wrist][unit]")
{
    for (const auto format : { PixelFormat::H264, PixelFormat::H265 })
    {
        CAPTURE(format);
        auto video_profile = profile;
        video_profile.format = format;
        const std::vector<uint8_t> independent =
            format == PixelFormat::H264 ? keyframe :
                                          std::vector<uint8_t>{ 0, 0, 0, 1, 0x40, 1, 0x55, 0, 0, 0, 1, 0x42, 1, 0x55,
                                                                0, 0, 0, 1, 0x44, 1, 0x55, 0, 0, 0, 1, 0x26, 1, 0x55 };
        const std::string marker = format == PixelFormat::H264 ?
                                       "EGO0000000000_L,timestamp_us=1792739705889228,frameId=1" :
                                       "EGO0000000000_L,timestamp_us=1792739755336474,frame_seq=1";
        const auto check = [&](const std::string& text, bool strip, bool trailing_bits = true)
        {
            Observations seen;
            FrameSink sink(video_profile, std::make_unique<MemorySink>(seen));
            std::vector<uint8_t> sei{ 0, 0, 0, 1 };
            if (format == PixelFormat::H264)
                sei.push_back(6);
            else
                sei.insert(sei.end(), { 0x4e, 1 });
            sei.push_back(5);
            sei.push_back(static_cast<uint8_t>(text.size()));
            sei.insert(sei.end(), text.begin(), text.end());
            if (trailing_bits)
                sei.push_back(0x80);
            auto bytes = independent;
            bytes.insert(bytes.end(), sei.begin(), sei.end());
            auto value = frame(0, bytes);
            value.profile = video_profile;
            std::vector<uint8_t> observed;
            sink.on_frame(value, [&](const auto& accepted) { observed = accepted.encoded_data; });
            REQUIRE(seen.media.size() == 1);
            REQUIRE(observed == (strip ? independent : bytes));
            REQUIRE(seen.metadata[0].encoded_data == observed);
            REQUIRE(seen.media[0].encoded_data == observed);
        };
        check(marker, true);
        check("ORBBEC,EGO_timestamp", false);
        check("unrelated user data", false);
        check("WRIST" + marker.substr(3), false);
        check(marker.substr(0, 3) + marker.substr(4), false);
        check(marker.substr(0, marker.size() - 1), false);
        check(marker + "extra", false);
        check(marker, false, false);
        auto invalid_timestamp = marker;
        invalid_timestamp[invalid_timestamp.find("timestamp_us=") + 13] = 'x';
        check(invalid_timestamp, false);
        auto wrong_field = marker;
        wrong_field.replace(
            wrong_field.find(",frame"), std::string::npos, format == PixelFormat::H264 ? ",frame_seq=1" : ",frameId=1");
        check(wrong_field, false);
    }
}

TEST_CASE("wrist H265 gate requires VPS SPS PPS and a keyframe", "[wrist][unit]")
{
    Observations seen;
    const VideoProfile h265{ PixelFormat::H265, 640, 480, 30 };
    FrameSink sink(h265, std::make_unique<MemorySink>(seen));
    const auto send = [&](uint64_t sequence, uint8_t nal)
    {
        auto value = frame(sequence, { 0, 0, 0, 1, nal, 1, 0x55 });
        value.profile = h265;
        sink.on_frame(value);
    };
    send(0, 0x26);
    REQUIRE(seen.media.empty());
    send(1, 0x40);
    send(2, 0x42);
    send(3, 0x44);
    REQUIRE(seen.media.empty());
    send(4, 0x26);
    REQUIRE(seen.media.size() == 1);
    REQUIRE(seen.media[0].encoded_data == std::vector<uint8_t>{ 0, 0, 0, 1, 0x40, 1, 0x55, 0, 0, 0, 1, 0x42, 1, 0x55,
                                                                0, 0, 0, 1, 0x44, 1, 0x55, 0, 0, 0, 1, 0x26, 1, 0x55 });
    send(6, 0x02);
    send(7, 0x26);
    REQUIRE(seen.media.size() == 1);
}

TEST_CASE("Orbbec Wrist preserves split H264 parameters in the accepted keyframe", "[wrist][unit]")
{
    Observations seen;
    FrameSink sink(profile, std::make_unique<MemorySink>(seen));
    sink.on_frame(frame(0, { 0, 0, 0, 1, 0x67, 1 }));
    sink.on_frame(frame(1, { 0, 0, 0, 1, 0x68, 1 }));
    sink.on_frame(frame(2, { 0, 0, 0, 1, 0x65, 1 }));
    REQUIRE(seen.media.size() == 1);
    REQUIRE(seen.media[0].encoded_data == keyframe);
    REQUIRE(seen.metadata[0].encoded_data == keyframe);
}

TEST_CASE("Orbbec Wrist notices a gap even when the next packet is stripped SEI", "[wrist][unit]")
{
    Observations seen;
    FrameSink sink(profile, std::make_unique<MemorySink>(seen));
    std::vector<uint8_t> sei{ 0, 0, 0, 1, 6, 5, 55 };
    const std::string marker = "EGO0000000000_L,timestamp_us=1792739705889228,frameId=1";
    sei.insert(sei.end(), marker.begin(), marker.end());
    sei.push_back(0x80);
    sink.on_frame(frame(0, keyframe));
    sink.on_frame(frame(2, sei));
    sink.on_frame(frame(3, predicted));
    REQUIRE(seen.media.size() == 1);
    sink.on_frame(frame(4, keyframe));
    REQUIRE(seen.media.size() == 2);
}

TEST_CASE("Orbbec Wrist H265 random access drops leading RASL pictures", "[wrist][unit]")
{
    Observations seen;
    auto video_profile = profile;
    video_profile.format = PixelFormat::H265;
    FrameSink sink(video_profile, std::make_unique<MemorySink>(seen));
    const auto send = [&](uint64_t sequence, const std::vector<uint8_t>& bytes)
    {
        auto value = frame(sequence, bytes);
        value.profile = video_profile;
        sink.on_frame(value);
    };
    send(0,
         { 0, 0, 0, 1, 0x40, 1, 0x55, 0, 0, 0, 1, 0x42, 1, 0x55, 0, 0, 0, 1, 0x44, 1, 0x55, 0, 0, 0, 1, 0x2a, 1, 0x55 });
    send(1, { 0, 0, 0, 1, 0x10, 1, 0x55 });
    send(2, { 0, 0, 0, 1, 0x12, 1, 0x55 });
    REQUIRE(seen.media.size() == 1);
    send(3, { 0, 0, 0, 1, 0x02, 1, 0x55 });
    REQUIRE(seen.media.size() == 2);
}

TEST_CASE("wrist batch preview observes gated video without losing IMU", "[wrist][unit]")
{
    Observations seen;
    FrameSink sink(profile, std::make_unique<MemorySink>(seen));
    CaptureBatch batch;
    batch.video = { frame(0, predicted), frame(1, keyframe), frame(2, predicted), frame(4, predicted),
                    frame(5, keyframe, 2) };
    batch.imu = { { ImuSensor::Accel, 7, { 100, 200, 9000 }, 1, false, { 1, 2, 3 }, 25 } };
    std::vector<VideoFrame> preview;
    sink.write(batch, [&](const VideoFrame& accepted) { preview.push_back(accepted); });
    sink.close_metadata();
    REQUIRE(preview.size() == 3);
    REQUIRE(preview.size() == seen.media.size());
    REQUIRE(preview[0].sequence == 1);
    REQUIRE(preview[1].sequence == 2);
    REQUIRE(preview[2].sequence == 5);
    REQUIRE(preview[2].capture_epoch == 2);
    for (size_t i = 0; i < preview.size(); ++i)
        REQUIRE(preview[i].encoded_data == seen.media[i].encoded_data);
    REQUIRE(seen.imu.size() == 1);
    REQUIRE(seen.imu[0].second.imu[0].sequence == 7);
}

TEST_CASE("wrist MJPG bypasses inter-frame gate and writes the same sidecar bytes", "[wrist][unit][recording]")
{
    const auto path = std::filesystem::temp_directory_path() /
                      ("isaaccapture-wrist-video-" + std::to_string(monotonic_now_ns()) + ".mjpg");
    struct Cleanup
    {
        std::filesystem::path path;
        ~Cleanup()
        {
            std::error_code error;
            std::filesystem::remove(path, error);
        }
    } cleanup{ path };
    Observations seen;
    auto value = frame(5, { 0xff, 0xd8, 0xff, 0xd9 });
    value.profile.format = PixelFormat::Mjpg;
    FrameSink sink(value.profile, std::make_unique<MemorySink>(seen), path.string());
    sink.on_frame(value);
    value.sequence = 20;
    sink.on_frame(value);
    sink.close_media();
    sink.close_metadata();
    sink.commit_metadata();
    std::ifstream file(path, std::ios::binary);
    const std::vector<uint8_t> bytes{ std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };
    auto expected = value.encoded_data;
    expected.insert(expected.end(), value.encoded_data.begin(), value.encoded_data.end());
    REQUIRE(bytes == expected);
    REQUIRE(seen.media.size() == 2);
    REQUIRE_THROWS(FrameSink(value.profile, nullptr, path.string()));
}

TEST_CASE("wrist frame sink batches independent IMU and separates epochs", "[wrist][unit]")
{
    Observations seen;
    FrameSink sink(profile, std::make_unique<MemorySink>(seen));
    CaptureBatch batch;
    for (uint64_t i = 0; i < 33; ++i)
        batch.imu.push_back({ ImuSensor::Accel, i, { i + 1, i + 2, 9000 }, 1, false, { 1, 2, 3 }, 25 });
    batch.imu.push_back({ ImuSensor::Gyro, 91, { 100, 200, 9000 }, 1, false, { 4, 5, 6 }, 26 });
    sink.write(batch);
    REQUIRE_FALSE(seen.imu.empty());
    REQUIRE(seen.imu[0].first == ImuSensor::Accel);
    REQUIRE(seen.imu[0].second.imu.size() == 32);
    batch.imu = { { ImuSensor::Accel, 0, { 1, 2, 9000 }, 2, false, { 7, 8, 9 }, 27 } };
    sink.write(batch);
    sink.close_media();
    sink.close_metadata();
    size_t accel = 0, gyro = 0;
    for (const auto& [sensor, part] : seen.imu)
    {
        REQUIRE(part.imu.size() <= 32);
        for (const auto& sample : part.imu)
        {
            REQUIRE(sample.sensor == sensor);
            REQUIRE(sample.capture_epoch == part.imu.front().capture_epoch);
            (sensor == ImuSensor::Accel ? accel : gyro)++;
        }
    }
    REQUIRE(accel == 34);
    REQUIRE(gyro == 1);
}

TEST_CASE("wrist composite finalizes every sink and prevents commit after failure or abort", "[wrist][unit]")
{
    Observations first, second;
    second.fail_close = true;
    std::vector<std::unique_ptr<IMetadataSink>> outputs;
    outputs.push_back(std::make_unique<MemorySink>(first));
    outputs.push_back(std::make_unique<MemorySink>(second));
    FrameSink sink(profile, compose_metadata_sinks(std::move(outputs)));
    sink.on_frame(frame(1, keyframe));
    REQUIRE(first.media.size() == 1);
    REQUIRE(second.metadata.size() == 1);
    REQUIRE_THROWS(sink.commit_metadata());
    sink.close_media();
    REQUIRE_THROWS(sink.close_metadata());
    REQUIRE(first.closed == 1);
    REQUIRE(second.closed == 1);
    REQUIRE_THROWS(sink.commit_metadata());
    sink.abort_metadata();
    REQUIRE(first.aborted == 1);
    REQUIRE(second.aborted == 1);
    REQUIRE(first.committed == 0);
    REQUIRE(second.committed == 0);
    REQUIRE_THROWS(sink.commit_metadata());
}
