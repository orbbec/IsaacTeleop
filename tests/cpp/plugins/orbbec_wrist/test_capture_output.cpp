// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <deviceio_trackers/orbbec_wrist_frame_metadata_tracker.hpp>
#include <deviceio_trackers/orbbec_wrist_imu_tracker.hpp>
#include <mcap/reader.hpp>
#include <mcap/recording_traits.hpp>
#include <mcap/tracker_channels.hpp>
#include <orbbec_wrist/core/metadata_sink.hpp>
#include <replay_trackers/replay_deviceio_factory.hpp>
#include <schema/orbbec_wrist_camera_bfbs_generated.h>
#include <schema/orbbec_wrist_imu_bfbs_generated.h>
#include <schema/serialized.hpp>

#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace plugins::wrist;
namespace
{
CaptureBatch data()
{
    CaptureBatch batch;
    batch.video.push_back({ { PixelFormat::H265, 1600, 1200, 30 },
                            42,
                            { 123456, 654321, 1000000000 },
                            7,
                            false,
                            { 0, 0, 0, 1, 0x40, 1, 0x55, 0, 0, 0, 1, 0x42, 1, 0x55,
                              0, 0, 0, 1, 0x44, 1, 0x55, 0, 0, 0, 1, 0x26, 1, 0x55 } });
    batch.imu.push_back({ ImuSensor::Accel, 3, { 123450, 654320, 999999998 }, 7, false, { 1, 2, 3 }, 21 });
    batch.imu.push_back({ ImuSensor::Gyro, 9, { 123455, 654322, 999999999 }, 7, true, { 4, 5, 6 }, 22 });
    return batch;
}
DeviceInfo device()
{
    return DeviceInfo{ .video = data().video.front().profile };
}
void close_output(FrameSink& sink)
{
    sink.close_media();
    sink.close_metadata();
}
struct TrackerSession : core::ITrackerSession
{
    std::vector<std::pair<const core::ITracker*, const core::ITrackerImpl*>> trackers;

    const core::ITrackerImpl& get_tracker_impl(const core::ITracker& tracker) const override
    {
        for (const auto& [key, value] : trackers)
            if (key == &tracker)
                return *value;
        throw std::out_of_range("Tracker not registered in test session");
    }
};

struct OutputPath
{
    std::filesystem::path directory =
        std::filesystem::temp_directory_path() / ("isaaccapture-wrist-test-" + std::to_string(monotonic_now_ns()));
    std::filesystem::path file = directory / "capture.mcap";
    ~OutputPath()
    {
        std::error_code error;
        std::filesystem::remove_all(directory, error);
    }
};
} // namespace

TEST_CASE("wrist payload mapping preserves packets, independent IMU and clocks", "[wrist][unit]")
{
    auto batch = data();
    const auto metadata = frame_payload(batch.video[0]);
    REQUIRE(metadata.stream == core::OrbbecWristCameraStream_Color);
    REQUIRE(metadata.width == 1600);
    REQUIRE(metadata.pixel_format == core::OrbbecWristPixelFormat_H265);
    REQUIRE(metadata.encoded_bytes == 28);
    REQUIRE(metadata.device_timestamp_us == 123456);
    REQUIRE(metadata.system_timestamp_us == 654321);
    REQUIRE(metadata.retrieved_monotonic_ns == 1000000000);
    const auto media = encoded_frame_payload(batch.video[0]);
    batch.video.clear();
    auto bytes = core::pack<core::OrbbecWristEncodedVideoFrame>(media);
    REQUIRE(bytes->encoded_data()->size() == 28);
    REQUIRE(bytes->sequence_number() == metadata.sequence_number);
    REQUIRE(bytes->capture_epoch() == metadata.capture_epoch);
    REQUIRE(bytes->stream() == metadata.stream);
    REQUIRE(bytes->width() == metadata.width);
    REQUIRE(bytes->height() == metadata.height);
    REQUIRE(bytes->fps() == metadata.fps);
    REQUIRE(bytes->pixel_format() == metadata.pixel_format);
    const auto accel = imu_payload(batch, ImuSensor::Accel, {});
    const auto gyro = imu_payload(batch, ImuSensor::Gyro, {});
    REQUIRE(accel.samples.size() == 1);
    REQUIRE(gyro.samples.size() == 1);
    REQUIRE(accel.samples[0].sequence_number() == 3);
    REQUIRE(gyro.samples[0].sequence_number() == 9);
    REQUIRE(gyro.samples[0].discontinuity());
}

TEST_CASE("wrist metadata serialization stays small independently of media size", "[wrist][unit]")
{
    auto frame = data().video.front();
    const auto small_metadata = core::pack<core::OrbbecWristFrameMetadata>(frame_payload(frame));
    frame.encoded_data.assign(1024 * 1024, 0x55);
    const auto large_metadata = core::pack<core::OrbbecWristFrameMetadata>(frame_payload(frame));
    const auto media = core::pack<core::OrbbecWristEncodedVideoFrame>(encoded_frame_payload(frame));
    REQUIRE(large_metadata->encoded_bytes() == frame.encoded_data.size());
    REQUIRE(large_metadata.buffer().size() == small_metadata.buffer().size());
    REQUIRE(large_metadata.buffer().size() <= 4096);
    REQUIRE(media->encoded_data()->size() == frame.encoded_data.size());
}

TEST_CASE("wrist MCAP round trip retains media, schema and timestamp domains", "[wrist][unit][recording]")
{
    OutputPath path;
    auto output = create_frame_sink({ .mcap_filename = path.file.string(), .embedded_media = true }, {}, device());
    output->write(data());
    REQUIRE_FALSE(std::filesystem::exists(path.file));
    close_output(*output);
    close_output(*output);
    output->commit_metadata();
    output->commit_metadata();
    REQUIRE(std::filesystem::exists(path.file));
    REQUIRE_FALSE(std::filesystem::exists(path.file.string() + ".partial"));
    mcap::McapReader reader;
    REQUIRE(reader.open(path.file.string()).ok());
    size_t metadata = 0, video = 0, imu = 0;
    for (const auto& message : reader.readMessages())
    {
        REQUIRE(message.schema);
        const auto* bytes = reinterpret_cast<const uint8_t*>(message.message.data);
        if (message.schema->name == core::OrbbecWristFrameMetadataRecord::GetFullyQualifiedName())
        {
            flatbuffers::Verifier verifier(bytes, message.message.dataSize);
            REQUIRE(verifier.VerifyBuffer<core::OrbbecWristFrameMetadataRecord>());
            const auto* record = flatbuffers::GetRoot<core::OrbbecWristFrameMetadataRecord>(bytes);
            REQUIRE(record->data()->stream() == core::OrbbecWristCameraStream_Color);
            REQUIRE(record->data()->encoded_bytes() == 28);
            REQUIRE(record->data()->sequence_number() == 42);
            REQUIRE(record->data()->capture_epoch() == 7);
            REQUIRE(record->data()->system_timestamp_us() == 654321);
            REQUIRE(record->timestamp()->sample_time_raw_device_clock() == 123456000);
            REQUIRE(record->timestamp()->sample_time_local_common_clock() == 1000000000);
            REQUIRE(message.message.logTime >= 1000000000);
            REQUIRE(record->timestamp()->available_time_local_common_clock() == message.message.logTime);
            REQUIRE(message.channel->topic == "wrist_camera/Color");
            ++metadata;
        }
        else if (message.schema->name == core::OrbbecWristEncodedVideoFrameRecord::GetFullyQualifiedName())
        {
            flatbuffers::Verifier verifier(bytes, message.message.dataSize);
            REQUIRE(verifier.VerifyBuffer<core::OrbbecWristEncodedVideoFrameRecord>());
            const auto* record = flatbuffers::GetRoot<core::OrbbecWristEncodedVideoFrameRecord>(bytes);
            REQUIRE(record->data()->stream() == core::OrbbecWristCameraStream_Color);
            REQUIRE(record->data()->sequence_number() == 42);
            REQUIRE(record->data()->capture_epoch() == 7);
            REQUIRE(record->data()->width() == 1600);
            REQUIRE(record->data()->height() == 1200);
            REQUIRE(record->data()->fps() == 30);
            REQUIRE(record->data()->pixel_format() == core::OrbbecWristPixelFormat_H265);
            const auto* encoded = record->data()->encoded_data();
            REQUIRE(encoded);
            REQUIRE(std::vector<uint8_t>(encoded->begin(), encoded->end()) == data().video.front().encoded_data);
            REQUIRE(record->timestamp()->sample_time_raw_device_clock() == 123456000);
            REQUIRE(record->timestamp()->sample_time_local_common_clock() == 1000000000);
            REQUIRE(record->timestamp()->available_time_local_common_clock() == message.message.logTime);
            REQUIRE(message.channel->topic == "orbbec_wrist_media/Color");
            ++video;
        }
        else
        {
            REQUIRE(message.schema->name == core::OrbbecWristImuBatchRecord::GetFullyQualifiedName());
            flatbuffers::Verifier verifier(bytes, message.message.dataSize);
            REQUIRE(verifier.VerifyBuffer<core::OrbbecWristImuBatchRecord>());
            const auto* record = flatbuffers::GetRoot<core::OrbbecWristImuBatchRecord>(bytes);
            REQUIRE(record->data()->samples()->size() == 1);
            REQUIRE(message.channel->topic == (record->data()->sensor() == core::OrbbecWristImuSensor_Accel ?
                                                   "wrist_camera/Accel/Imu" :
                                                   "wrist_camera/Gyro/Imu"));
            ++imu;
        }
    }
    REQUIRE(metadata == 1);
    REQUIRE(video == 1);
    REQUIRE(imu == 2);
    const core::RecordedSchemas recorded(reader);
    auto replay_reader = std::make_unique<mcap::McapReader>();
    REQUIRE(replay_reader->open(path.file.string()).ok());
    core::McapTrackerViewers<core::OrbbecWristFrameMetadataRecord> replay(
        std::move(replay_reader), "wrist_camera", { "Color" }, recorded);
    const auto replayed = replay.read(0);
    REQUIRE(replayed);
    REQUIRE(replayed->data()->sequence_number() == 42);
    REQUIRE(replayed->data()->encoded_bytes() == 28);
    REQUIRE_FALSE(replay.read(0));
    auto media_reader = std::make_unique<mcap::McapReader>();
    REQUIRE(media_reader->open(path.file.string()).ok());
    core::McapTrackerViewers<core::OrbbecWristEncodedVideoFrameRecord> media_replay(
        std::move(media_reader), "orbbec_wrist_media", { "Color" }, recorded);
    const auto replayed_media = media_replay.read(0);
    REQUIRE(replayed_media);
    REQUIRE(replayed_media->data()->sequence_number() == replayed->data()->sequence_number());
    REQUIRE(replayed_media->data()->capture_epoch() == replayed->data()->capture_epoch());
    REQUIRE(replayed_media->data()->encoded_data()->size() == replayed->data()->encoded_bytes());
    REQUIRE_FALSE(media_replay.read(0));
    REQUIRE_THROWS(create_frame_sink({ .mcap_filename = path.file.string(), .embedded_media = true }, {}, device()));
}

TEST_CASE("wrist MCAP metadata and media callbacks write independent channels", "[wrist][unit][recording]")
{
    const bool media_only = GENERATE(false, true);
    OutputPath path;
    auto output = make_mcap_metadata_sink(path.file.string(), {}, device(), true);
    const auto frame = data().video.front();
    if (media_only)
        output->on_encoded_video_frame(frame);
    else
        output->on_frame_metadata(frame);
    output->close();
    output->commit();
    mcap::McapReader reader;
    REQUIRE(reader.open(path.file.string()).ok());
    size_t messages = 0;
    for (const auto& message : reader.readMessages())
    {
        REQUIRE(message.schema);
        REQUIRE(message.channel->topic == (media_only ? "orbbec_wrist_media/Color" : "wrist_camera/Color"));
        REQUIRE(message.schema->name == (media_only ? core::OrbbecWristEncodedVideoFrameRecord::GetFullyQualifiedName() :
                                                      core::OrbbecWristFrameMetadataRecord::GetFullyQualifiedName()));
        ++messages;
    }
    REQUIRE(messages == 1);
}

TEST_CASE("wrist generated replay routes independent consumers and skips tracked snapshots", "[wrist][unit][replay]")
{
    const bool session_recording = GENERATE(false, true);
    const int selection = GENERATE(0, 1, 2);
    std::vector<core::OrbbecWristImuSensor> sensors = { core::OrbbecWristImuSensor_Accel,
                                                        core::OrbbecWristImuSensor_Gyro };
    if (selection == 1)
        sensors = { core::OrbbecWristImuSensor_Gyro, core::OrbbecWristImuSensor_Accel };
    else if (selection == 2)
        sensors = { core::OrbbecWristImuSensor_Gyro };

    OutputPath path;
    if (session_recording)
    {
        std::filesystem::create_directories(path.directory);
        mcap::McapWriter writer;
        mcap::McapWriterOptions options("wrist-test");
        options.compression = mcap::Compression::None;
        REQUIRE(writer.open(path.file.string(), options).ok());
        const auto& video_names = core::OrbbecWristFrameMetadataRecordingTraits::recording_channels;
        const auto& imu_names = core::OrbbecWristImuRecordingTraits::recording_channels;
        const std::vector<std::string> video_channels(video_names.begin(), video_names.end());
        const std::vector<std::string> imu_channels(imu_names.begin(), imu_names.end());
        core::McapTrackerChannels<core::OrbbecWristFrameMetadataRecord> video(writer, "wrist_camera", video_channels);
        core::McapTrackerChannels<core::OrbbecWristImuBatchRecord> accel(writer, "wrist_camera/Accel", imu_channels);
        core::McapTrackerChannels<core::OrbbecWristImuBatchRecord> gyro(writer, "wrist_camera/Gyro", imu_channels);
        const core::DeviceDataTimestamp timestamp(1000000001, 1000000000, 123456000);
        for (uint64_t step = 0; step < 2; ++step)
        {
            auto batch = data();
            batch.video.front().sequence -= 1 - step;
            for (auto& sample : batch.imu)
                sample.sequence -= 1 - step;
            const auto frame = frame_payload(batch.video.front());
            const auto video_record = core::pack_record<core::OrbbecWristFrameMetadataRecord>(&frame, timestamp);
            video.write(0, video_record);
            if (step == 1)
                video.write(1, video_record);
            for (const auto sensor : { ImuSensor::Accel, ImuSensor::Gyro })
            {
                const auto payload = imu_payload(batch, sensor, {});
                const auto record = core::pack_record<core::OrbbecWristImuBatchRecord>(&payload, timestamp);
                auto& channel = sensor == ImuSensor::Accel ? accel : gyro;
                channel.write(0, record);
                if (step == 1)
                    channel.write(1, record);
            }
        }
        writer.close();
    }
    else
    {
        auto output = create_frame_sink({ .mcap_filename = path.file.string(), .embedded_media = true }, {}, device());
        output->write(data());
        close_output(*output);
        output->commit_metadata();
    }

    core::OrbbecWristFrameMetadataTracker camera("wrist/Color");
    std::vector<std::unique_ptr<core::OrbbecWristImuTracker>> imu_trackers;
    std::vector<std::pair<const core::ITracker*, std::string>> names = { { &camera, "wrist_camera" } };
    for (const auto sensor : sensors)
    {
        const std::string name = sensor == core::OrbbecWristImuSensor_Accel ? "Accel" : "Gyro";
        imu_trackers.push_back(std::make_unique<core::OrbbecWristImuTracker>("wrist/" + name));
        names.emplace_back(imu_trackers.back().get(), "wrist_camera/" + name);
    }
    core::ReplayDeviceIOFactory factory(path.file.string(), names);
    auto camera_impl = factory.create_tracker_impl(camera);
    std::vector<std::unique_ptr<core::ITrackerImpl>> imu_impls;
    TrackerSession session;
    session.trackers = { { &camera, camera_impl.get() } };
    REQUIRE_FALSE(camera.get_data(session));
    for (const auto& tracker : imu_trackers)
    {
        imu_impls.push_back(factory.create_tracker_impl(*tracker));
        session.trackers.emplace_back(tracker.get(), imu_impls.back().get());
        REQUIRE_FALSE(tracker->get_data(session));
    }

    camera_impl->update(0);
    for (auto& impl : imu_impls)
        impl->update(0);
    const auto snapshot = camera.get_data(session);
    const uint64_t decrement = session_recording ? 1 : 0;
    REQUIRE(snapshot);
    REQUIRE(snapshot->stream() == core::OrbbecWristCameraStream_Color);
    REQUIRE(snapshot->sequence_number() == 42 - decrement);
    REQUIRE(snapshot->capture_epoch() == 7);
    REQUIRE(snapshot->device_timestamp_us() == 123456);
    REQUIRE(snapshot->encoded_bytes() == 28);
    std::vector<core::Serialized<core::OrbbecWristImuBatch>> imu_snapshots;
    for (size_t index = 0; index < sensors.size(); ++index)
    {
        const auto sample = imu_trackers[index]->get_data(session);
        REQUIRE(sample);
        REQUIRE(sample->sensor() == sensors[index]);
        REQUIRE(sample->samples()->size() == 1);
        REQUIRE(sample->samples()->Get(0)->sequence_number() ==
                (sensors[index] == core::OrbbecWristImuSensor_Accel ? 3 : 9) - decrement);
        imu_snapshots.push_back(sample);
    }

    if (session_recording)
    {
        camera_impl->update(0);
        REQUIRE(camera.get_data(session));
        REQUIRE(camera.get_data(session)->sequence_number() == 42);
        for (size_t index = 0; index < imu_impls.size(); ++index)
        {
            imu_impls[index]->update(0);
            const auto sample = imu_trackers[index]->get_data(session);
            REQUIRE(sample);
            REQUIRE(sample->samples()->Get(0)->sequence_number() ==
                    (sensors[index] == core::OrbbecWristImuSensor_Accel ? 3 : 9));
        }
    }
    camera_impl->update(0);
    for (auto& impl : imu_impls)
        impl->update(0);
    REQUIRE_FALSE(camera.get_data(session));
    for (const auto& tracker : imu_trackers)
        REQUIRE_FALSE(tracker->get_data(session));
    camera_impl.reset();
    imu_impls.clear();
    REQUIRE(snapshot->sequence_number() == 42 - decrement);
    for (size_t index = 0; index < sensors.size(); ++index)
        REQUIRE(imu_snapshots[index]->samples()->Get(0)->sequence_number() ==
                (sensors[index] == core::OrbbecWristImuSensor_Accel ? 3 : 9) - decrement);
}

TEST_CASE("wrist failed capture stays partial and cannot replace a previous output", "[wrist][unit][recording]")
{
    OutputPath path;
    {
        auto output = create_frame_sink({ .mcap_filename = path.file.string(), .embedded_media = true }, {}, device());
        REQUIRE_THROWS(output->commit_metadata());
        output->write(data());
        output->abort_metadata();
        close_output(*output);
        REQUIRE_THROWS(output->commit_metadata());
    }
    REQUIRE_FALSE(std::filesystem::exists(path.file));
    REQUIRE(std::filesystem::exists(path.file.string() + ".partial"));
    REQUIRE_THROWS(create_frame_sink({ .mcap_filename = path.file.string(), .embedded_media = true }, {}, device()));
}

TEST_CASE("wrist promotion preserves a destination created after capture started", "[wrist][unit][recording]")
{
    OutputPath path;
    auto output = create_frame_sink({ .mcap_filename = path.file.string(), .embedded_media = true }, {}, device());
    output->write(data());
    close_output(*output);
    {
        std::ofstream previous(path.file);
        previous << "previous-capture";
    }
    REQUIRE_THROWS(output->commit_metadata());
    std::ifstream previous(path.file);
    std::string content;
    previous >> content;
    REQUIRE(content == "previous-capture");
    REQUIRE(std::filesystem::exists(path.file.string() + ".partial"));
}

TEST_CASE("wrist default MCAP records metadata without embedded media", "[wrist][unit][recording]")
{
    OutputPath path;
    auto output = create_frame_sink({ .mcap_filename = path.file.string() }, {}, device());
    output->write(data());
    output->metadata_sink()->on_capture_status(CaptureState::Starting, {});
    output->metadata_sink()->on_capture_status(CaptureState::Closed, {});
    close_output(*output);
    output->commit_metadata();
    mcap::McapReader reader;
    REQUIRE(reader.open(path.file.string()).ok());
    size_t metadata = 0, imu = 0, diagnostics = 0;
    for (const auto& message : reader.readMessages())
    {
        REQUIRE(message.channel->topic != "orbbec_wrist_media/Color");
        if (message.channel->topic == "wrist_camera/Color")
            ++metadata;
        else if (message.channel->topic == "wrist_camera/Diagnostics")
            ++diagnostics;
        else
            ++imu;
    }
    REQUIRE(metadata == 1);
    REQUIRE(imu == 2);
    REQUIRE(diagnostics == 2);
}

TEST_CASE("wrist media spool validates destinations before opening outputs", "[wrist][spool][unit]")
{
    OutputPath path;
    OutputConfig output;
    output.embedded_media = true;
    REQUIRE_THROWS_AS(validate_output_config(output), std::invalid_argument);
    output.mcap_media_spool = path.file.string();
    REQUIRE_THROWS_AS(validate_output_config(output), std::invalid_argument);
    output.collection_prefix = "wrist/left";
    REQUIRE_NOTHROW(validate_output_config(output));
    output.embedded_media = false;
    REQUIRE_THROWS_AS(validate_output_config(output), std::invalid_argument);
    output.embedded_media = true;
    output.video_output = path.file.string() + ".partial";
    REQUIRE_THROWS_AS(validate_output_config(output), std::invalid_argument);
    REQUIRE_FALSE(std::filesystem::exists(path.directory));
}

TEST_CASE("wrist media spool commits namespaced metadata media IMU and terminal status", "[wrist][spool][unit]")
{
    const auto prefix = GENERATE(std::string("wrist/left"), std::string("wrist/right"));
    const auto abort = GENERATE(false, true);
    OutputPath path;
    auto info = device();
    info.serial_number = prefix;
    info.calibration_json = R"({"color":null})";
    auto metadata = make_mcap_metadata_sink(path.file.string(), {}, info, true, prefix, "wrist_root");
    FrameSink sink(info.video, std::move(metadata), {});
    sink.write(data());
    sink.metadata_sink()->on_capture_status(CaptureState::Closed, {});
    close_output(sink);
    REQUIRE_FALSE(std::filesystem::exists(path.file));
    REQUIRE(std::filesystem::exists(path.file.string() + ".partial"));
    if (abort)
    {
        sink.abort_metadata();
        REQUIRE_THROWS(sink.commit_metadata());
        REQUIRE_FALSE(std::filesystem::exists(path.file));
        return;
    }
    sink.commit_metadata();
    REQUIRE_FALSE(std::filesystem::exists(path.file.string() + ".partial"));
    mcap::McapReader reader;
    REQUIRE(reader.open(path.file.string()).ok());
    size_t count = 0;
    for (const auto& view : reader.readMessages())
    {
        ++count;
        REQUIRE(view.channel->topic.starts_with(prefix + "/"));
        if (view.schema->name == core::OrbbecWristEncodedVideoFrameRecord::GetFullyQualifiedName())
        {
            REQUIRE(view.channel->topic == prefix + "/EncodedVideo/Color");
            const auto* record = flatbuffers::GetRoot<core::OrbbecWristEncodedVideoFrameRecord>(view.message.data);
            const auto expected = data().video.front().encoded_data;
            REQUIRE(std::vector<uint8_t>(
                        record->data()->encoded_data()->begin(), record->data()->encoded_data()->end()) == expected);
        }
    }
    REQUIRE(count == 5); // Color metadata, video, Accel, Gyro and Closed.
}
