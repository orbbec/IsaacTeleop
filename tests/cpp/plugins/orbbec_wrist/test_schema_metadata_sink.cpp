// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <deviceio_trackers/orbbec_wrist_frame_metadata_tracker.hpp>
#include <deviceio_trackers/orbbec_wrist_imu_tracker.hpp>
#include <live_trackers/live_deviceio_factory.hpp>
#include <orbbec_wrist/core/metadata_sink.hpp>
#include <pusherio/schema_pusher.hpp>
#include <schema/plugin_device_status_generated.h>

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>

using namespace plugins::wrist;

namespace
{
// Exercise the real SchemaPusher and publishing thread at the runtime boundary.
struct Runtime
{
    struct Sample
    {
        std::vector<uint8_t> bytes;
        XrTime timestamp;
        uint64_t raw_timestamp;
    };
    struct Collection
    {
        Runtime* runtime;
        std::string tensor;
        uint32_t size;
        std::vector<Sample> samples;
        bool destroyed = false;
    };
    std::mutex mutex;
    std::condition_variable changed;
    std::map<std::string, Collection> collections;
    bool fail_push = false;
    std::string fail_create_id, fail_push_id, blocked_id, missing_function;
    bool push_blocked = false, release_push = false, block_timed_out = false;
    std::vector<std::string> events;
    void block(const std::string& id)
    {
        std::lock_guard<std::mutex> lock(mutex);
        blocked_id = id;
        push_blocked = release_push = false;
    }
    bool wait_blocked()
    {
        std::unique_lock<std::mutex> lock(mutex);
        return changed.wait_for(lock, std::chrono::seconds(3), [&] { return push_blocked; });
    }
    void unblock()
    {
        std::lock_guard<std::mutex> lock(mutex);
        release_push = true;
        changed.notify_all();
    }
    bool wait_status(const std::string& id, core::PluginDeviceState state)
    {
        std::unique_lock<std::mutex> lock(mutex);
        return changed.wait_for(lock, std::chrono::seconds(3),
                                [&]
                                {
                                    const auto entry = collections.find(id);
                                    if (entry == collections.end() || entry->second.samples.empty())
                                        return false;
                                    const auto* status = flatbuffers::GetRoot<core::PluginDeviceStatusSnapshot>(
                                        entry->second.samples.back().bytes.data());
                                    return status->devices()->Get(0)->state() == state;
                                });
    }
    struct List
    {
        Runtime* runtime;
        std::vector<std::pair<std::string, Collection*>> entries;
    };

    core::OpenXRSessionHandles handles()
    {
        return { reinterpret_cast<XrInstance>(this), reinterpret_cast<XrSession>(this), XR_NULL_HANDLE, dispatch };
    }
    static XrResult XRAPI_CALL create(XrSession session,
                                      const XrPushTensorCollectionCreateInfoNV* info,
                                      XrPushTensorCollectionCreateResultNV*,
                                      XrPushTensorCollectionNV* result)
    {
        auto& runtime = *reinterpret_cast<Runtime*>(session);
        std::lock_guard<std::mutex> lock(runtime.mutex);
        if (runtime.fail_create_id == info->data.identifier)
            return XR_ERROR_RUNTIME_FAILURE;
        auto [entry, inserted] = runtime.collections.try_emplace(
            info->data.identifier,
            Collection{ &runtime, info->tensors[0].properties.identifier, info->data.totalSampleSize, {} });
        if (!inserted)
            return XR_ERROR_RUNTIME_FAILURE;
        *result = reinterpret_cast<XrPushTensorCollectionNV>(&entry->second);
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL destroy(XrPushTensorCollectionNV handle)
    {
        auto& collection = *reinterpret_cast<Collection*>(handle);
        std::lock_guard<std::mutex> lock(collection.runtime->mutex);
        collection.destroyed = true;
        collection.runtime->events.push_back("destroy:" + collection.tensor);
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL push(XrPushTensorCollectionNV handle, const XrPushTensorCollectionDataNV* data)
    {
        auto& collection = *reinterpret_cast<Collection*>(handle);
        auto& runtime = *collection.runtime;
        std::unique_lock<std::mutex> lock(runtime.mutex);
        if (runtime.blocked_id == collection.tensor && !runtime.release_push)
        {
            runtime.push_blocked = true;
            runtime.changed.notify_all();
            if (!runtime.changed.wait_for(lock, std::chrono::seconds(3), [&] { return runtime.release_push; }))
                runtime.block_timed_out = true;
        }
        if (runtime.fail_push || runtime.fail_push_id == collection.tensor)
            return XR_ERROR_RUNTIME_FAILURE;
        runtime.events.push_back("push:" + collection.tensor);
        collection.samples.push_back(
            { { data->buffer, data->buffer + data->bufferSize }, data->timestamp, data->rawDeviceTimestamp });
        collection.runtime->changed.notify_all();
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL from_timespec(XrInstance, const timespec* value, XrTime* result)
    {
        if (value->tv_sec < 0 || value->tv_nsec < 0 || value->tv_nsec >= 1'000'000'000)
            return XR_ERROR_TIME_INVALID;
        *result = value->tv_sec * 1'000'000'000LL + value->tv_nsec;
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL to_timespec(XrInstance, XrTime value, timespec* result)
    {
        result->tv_sec = value / 1'000'000'000LL;
        result->tv_nsec = value % 1'000'000'000LL;
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL generation(XrSession session, uint64_t* result)
    {
        auto& runtime = *reinterpret_cast<Runtime*>(session);
        std::lock_guard<std::mutex> lock(runtime.mutex);
        *result = runtime.collections.size();
        for (const auto& [id, collection] : runtime.collections)
            *result += collection.destroyed;
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL update_list(XrTensorListNV handle)
    {
        auto& list = *reinterpret_cast<List*>(handle);
        std::lock_guard<std::mutex> lock(list.runtime->mutex);
        list.entries.clear();
        for (auto& [id, collection] : list.runtime->collections)
            if (!collection.destroyed)
                list.entries.emplace_back(id, &collection);
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL create_list(XrSession session, const XrCreateTensorListInfoNV*, XrTensorListNV* result)
    {
        *result = reinterpret_cast<XrTensorListNV>(new List{ reinterpret_cast<Runtime*>(session), {} });
        return update_list(*result);
    }
    static XrResult XRAPI_CALL destroy_list(XrTensorListNV handle)
    {
        delete reinterpret_cast<List*>(handle);
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL list_properties(XrTensorListNV handle, XrSystemTensorListPropertiesNV* result)
    {
        result->tensorCollectionCount = reinterpret_cast<List*>(handle)->entries.size();
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL collection_properties(XrTensorListNV handle,
                                                     uint32_t index,
                                                     XrTensorCollectionPropertiesNV* result)
    {
        const auto& [id, collection] = reinterpret_cast<List*>(handle)->entries.at(index);
        std::strncpy(result->data.identifier, id.c_str(), XR_MAX_TENSOR_IDENTIFIER_SIZE - 1);
        result->data.totalSampleSize = result->sampleBatchStride = collection->size;
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL read(XrTensorListNV handle, const XrTensorDataRetrievalInfoNV* info, XrTensorDataNV* data)
    {
        auto& list = *reinterpret_cast<List*>(handle);
        std::lock_guard<std::mutex> lock(list.runtime->mutex);
        const auto& collection = *list.entries.at(info->tensorCollectionIndex).second;
        if (collection.destroyed)
            return XR_ERROR_TENSOR_LOST_NV;
        data->writtenSampleCount = 0;
        if (info->startSampleIndex < 0 || static_cast<size_t>(info->startSampleIndex) >= collection.samples.size())
            return XR_SUCCESS;
        const auto& sample = collection.samples.at(info->startSampleIndex);
        if (data->bufferCapacity < sample.bytes.size() || data->metadataCapacity < 1)
            return XR_ERROR_SIZE_INSUFFICIENT;
        std::memcpy(data->buffer, sample.bytes.data(), sample.bytes.size());
        data->metadataArray[0].sampleIndex = info->startSampleIndex;
        data->metadataArray[0].timestamp = sample.timestamp;
        data->metadataArray[0].arrivalTimestamp = sample.timestamp + 100;
        data->metadataArray[0].rawDeviceTimestamp = sample.raw_timestamp;
        data->writtenSampleCount = 1;
        return XR_SUCCESS;
    }
    static XrResult XRAPI_CALL dispatch(XrInstance instance, const char* name, PFN_xrVoidFunction* function)
    {
        if (reinterpret_cast<Runtime*>(instance)->missing_function == name)
        {
            *function = nullptr;
            return XR_ERROR_FUNCTION_UNSUPPORTED;
        }
        if (std::strcmp(name, "xrCreatePushTensorCollectionNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(create);
        else if (std::strcmp(name, "xrDestroyPushTensorCollectionNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(destroy);
        else if (std::strcmp(name, "xrPushTensorCollectionDataNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(push);
        else if (std::strcmp(name, "xrConvertTimespecTimeToTimeKHR") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(from_timespec);
        else if (std::strcmp(name, "xrConvertTimeToTimespecTimeKHR") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(to_timespec);
        else if (std::strcmp(name, "xrGetTensorListLatestGenerationNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(generation);
        else if (std::strcmp(name, "xrCreateTensorListNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(create_list);
        else if (std::strcmp(name, "xrDestroyTensorListNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(destroy_list);
        else if (std::strcmp(name, "xrUpdateTensorListNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(update_list);
        else if (std::strcmp(name, "xrGetTensorListPropertiesNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(list_properties);
        else if (std::strcmp(name, "xrGetTensorCollectionPropertiesNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(collection_properties);
        else if (std::strcmp(name, "xrGetTensorDataNV") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(read);
        else
            return XR_ERROR_FUNCTION_UNSUPPORTED;
        return XR_SUCCESS;
    }
    bool wait_for_samples(const std::string& id, size_t count)
    {
        std::unique_lock<std::mutex> lock(mutex);
        return changed.wait_for(lock, std::chrono::seconds(3),
                                [&] { return collections.count(id) && collections.at(id).samples.size() >= count; });
    }
};
}

TEST_CASE("wrist schema sink drains terminal status without queued frames", "[wrist][unit]")
{
    for (const auto terminal : { CaptureState::Closed, CaptureState::Faulted })
    {
        for (int iteration = 0; iteration < 20; ++iteration)
        {
            Runtime runtime;
            OutputConfig output;
            output.plugin_root_id = "/wrist";
            auto sink = make_schema_metadata_sink(output, {}, runtime.handles());
            sink->on_capture_status(CaptureState::Streaming, "");
            sink->on_capture_status(terminal, terminal == CaptureState::Faulted ? "camera lost" : "");
            sink->close();
            sink->close();
            REQUIRE(runtime.collections.count("/wrist/device_status") == 1);
            const auto& collection = runtime.collections.at("/wrist/device_status");
            REQUIRE(collection.destroyed);
            REQUIRE_FALSE(collection.samples.empty());
            const auto* status =
                flatbuffers::GetRoot<core::PluginDeviceStatusSnapshot>(collection.samples.back().bytes.data());
            REQUIRE(status->schema_version() == 1);
            REQUIRE(status->devices()->size() == 1);
            const auto* device = status->devices()->Get(0);
            CHECK(device->path()->str() == "/camera/wrist");
            CHECK(device->state() == (terminal == CaptureState::Closed ? core::PluginDeviceState_DISCONNECTED :
                                                                         core::PluginDeviceState_FAILED));
            CHECK(device->reason() == (terminal == CaptureState::Closed ? core::PluginDeviceReason_NO_HARDWARE_SIGNAL :
                                                                          core::PluginDeviceReason_DEVICE_ERROR));
            const auto error = device->error() ? device->error()->str() : std::string{};
            CHECK(error == (terminal == CaptureState::Faulted ? "camera lost" : ""));
        }
    }
}

TEST_CASE("wrist schema sinks isolate instances and drain metadata and IMU", "[wrist][unit]")
{
    Runtime runtime;
    OutputConfig left, right;
    left.collection_prefix = left.plugin_root_id = "/wrist/left";
    right.collection_prefix = right.plugin_root_id = "/wrist/right";
    auto first = make_schema_metadata_sink(left, {}, runtime.handles());
    auto second = make_schema_metadata_sink(right, {}, runtime.handles());
    VideoFrame frame;
    frame.sequence = 17;
    frame.time = { 1234, 5678, 9000 };
    frame.encoded_data = { 1, 2, 3 };
    first->on_frame_metadata(frame);
    first->on_capture_status(CaptureState::Closed, "closed");
    first->close();
    frame.sequence = 29;
    second->on_frame_metadata(frame);
    CaptureBatch batch;
    batch.imu.push_back({ ImuSensor::Accel, 2, frame.time, 1, false, { 1, 2, 3 }, 25 });
    second->on_imu_batch(batch, ImuSensor::Accel);
    batch.imu[0].sensor = ImuSensor::Gyro;
    second->on_imu_batch(batch, ImuSensor::Gyro);
    second->on_capture_status(CaptureState::Streaming, "");
    REQUIRE(runtime.wait_for_samples("/wrist/right/device_status", 1));
    second->close();
    for (const auto& [id, collection] : runtime.collections)
        CHECK(collection.destroyed);
    for (const auto& [id, sequence] :
         std::map<std::string, uint64_t>{ { "/wrist/left/Color", 17 }, { "/wrist/right/Color", 29 } })
    {
        const auto& samples = runtime.collections.at(id).samples;
        REQUIRE(samples.size() == 1);
        CHECK(samples[0].timestamp == 9000);
        CHECK(samples[0].raw_timestamp == 1'234'000);
        const auto* metadata = flatbuffers::GetRoot<core::OrbbecWristFrameMetadata>(samples[0].bytes.data());
        CHECK(metadata->sequence_number() == sequence);
        CHECK(metadata->encoded_bytes() == 3);
    }
    for (const auto& sensor : { "Accel", "Gyro" })
    {
        const auto& collection = runtime.collections.at(std::string("/wrist/right/") + sensor);
        REQUIRE(collection.samples.size() == 1);
        const auto* imu = flatbuffers::GetRoot<core::OrbbecWristImuBatch>(collection.samples[0].bytes.data());
        REQUIRE(imu->samples()->size() == 1);
        CHECK(imu->samples()->Get(0)->sequence_number() == 2);
    }
}

TEST_CASE("wrist schema sink supports video only and reports transport failures", "[wrist][unit]")
{
    Runtime runtime;
    runtime.fail_push = true;
    OutputConfig output;
    output.collection_prefix = "/wrist";
    CaptureConfig capture;
    capture.enable_imu = false;
    auto sink = make_schema_metadata_sink(output, capture, runtime.handles());
    sink->on_frame_metadata(VideoFrame{});
    CHECK_THROWS_WITH(sink->close(), Catch::Matchers::ContainsSubstring("Failed to push tensor data"));
    CHECK(runtime.collections.size() == 1);
    CHECK(runtime.collections.at("/wrist/Color").destroyed);
}

TEST_CASE("wrist published data reaches generated live consumers independently", "[wrist][unit]")
{
    Runtime runtime;
    // LiveDeviceIOFactory and its readers borrow handles; keep them alive through reader destruction.
    const auto handles = runtime.handles();
    OutputConfig left, right;
    left.collection_prefix = "/wrist/left";
    right.collection_prefix = "/wrist/right";
    auto first = make_schema_metadata_sink(left, {}, runtime.handles());
    auto second = make_schema_metadata_sink(right, {}, runtime.handles());
    struct Session : core::ITrackerSession
    {
        std::map<const core::ITracker*, std::unique_ptr<core::ITrackerImpl>> implementations;
        const core::ITrackerImpl& get_tracker_impl(const core::ITracker& tracker) const override
        {
            return *implementations.at(&tracker);
        }
        void update()
        {
            for (auto& [tracker, impl] : implementations)
                impl->update(10000);
        }
    } session;
    core::OrbbecWristFrameMetadataTracker left_color("/wrist/left/Color"), right_color("/wrist/right/Color");
    core::OrbbecWristImuTracker accel("/wrist/right/Accel"), gyro("/wrist/right/Gyro");
    core::LiveDeviceIOFactory factory(handles, nullptr, {});
    for (const core::ITracker* tracker : std::vector<const core::ITracker*>{ &left_color, &right_color, &accel, &gyro })
        session.implementations.emplace(tracker, factory.create_tracker_impl(*tracker));
    VideoFrame frame;
    frame.sequence = 1;
    frame.time = { 12, 34, 5678 };
    first->on_frame_metadata(frame);
    frame.sequence = 2;
    second->on_frame_metadata(frame);
    CaptureBatch batch;
    batch.imu.push_back({ ImuSensor::Accel, 7, frame.time, 1, false, { 1, 2, 3 }, 25 });
    second->on_imu_batch(batch, ImuSensor::Accel);
    batch.imu[0].sensor = ImuSensor::Gyro;
    batch.imu[0].sequence = 9;
    second->on_imu_batch(batch, ImuSensor::Gyro);
    for (const auto& id : { "/wrist/left/Color", "/wrist/right/Color", "/wrist/right/Accel", "/wrist/right/Gyro" })
        REQUIRE(runtime.wait_for_samples(id, 1));
    session.update();
    REQUIRE(left_color.get_data(session));
    REQUIRE(right_color.get_data(session));
    CHECK(left_color.get_data(session)->sequence_number() == 1);
    CHECK(right_color.get_data(session)->sequence_number() == 2);
    REQUIRE(accel.get_data(session));
    REQUIRE(gyro.get_data(session));
    CHECK(accel.get_data(session)->samples()->Get(0)->sequence_number() == 7);
    CHECK(gyro.get_data(session)->samples()->Get(0)->sequence_number() == 9);
    first->close();
    frame.sequence = 3;
    second->on_frame_metadata(frame);
    REQUIRE(runtime.wait_for_samples("/wrist/right/Color", 2));
    session.update();
    CHECK_FALSE(left_color.get_data(session));
    REQUIRE(right_color.get_data(session));
    CHECK(right_color.get_data(session)->sequence_number() == 3);
    second->close();
}


TEST_CASE("wrist matrix covers all six capture status mappings", "[wrist][unit]")
{
    const std::array states{ CaptureState::Closed,    CaptureState::Configured, CaptureState::Starting,
                             CaptureState::Streaming, CaptureState::Recovering, CaptureState::Faulted };
    for (size_t index = 0; index < states.size(); ++index)
    {
        INFO("X-STATE fake_dispatch index=" << index * 2);
        Runtime runtime;
        OutputConfig output;
        output.plugin_root_id = "/wrist/status_matrix";
        auto sink = make_schema_metadata_sink(output, {}, runtime.handles());
        const auto target = states[index];
        sink->on_capture_status(
            target == CaptureState::Streaming ? CaptureState::Faulted : CaptureState::Streaming, "prior");
        REQUIRE(runtime.wait_status(output.plugin_root_id + "/device_status", target == CaptureState::Streaming ?
                                                                                  core::PluginDeviceState_FAILED :
                                                                                  core::PluginDeviceState_CONNECTED));
        const auto before = monotonic_now_ns();
        sink->on_capture_status(target, target == CaptureState::Faulted ? "matrix failure" : "");
        const auto state = target == CaptureState::Closed    ? core::PluginDeviceState_DISCONNECTED :
                           target == CaptureState::Streaming ? core::PluginDeviceState_CONNECTED :
                           target == CaptureState::Faulted   ? core::PluginDeviceState_FAILED :
                                                               core::PluginDeviceState_DEGRADED;
        REQUIRE(runtime.wait_status(output.plugin_root_id + "/device_status", state));
        const auto after = monotonic_now_ns();
        sink->close();
        const auto& bytes = runtime.collections.at(output.plugin_root_id + "/device_status").samples.back().bytes;
        const auto* status = flatbuffers::GetRoot<core::PluginDeviceStatusSnapshot>(bytes.data());
        const auto* device = status->devices()->Get(0);
        CHECK(status->schema_version() == 1);
        CHECK(status->report_time_ns() >= before);
        CHECK(status->report_time_ns() <= after);
        CHECK(device->path()->str() == "/camera/wrist");
        CHECK((device->error() ? device->error()->str() : std::string{}) ==
              (target == CaptureState::Faulted ? "matrix failure" : ""));
        CHECK(device->state() == state);
        CHECK(device->reason() == (target == CaptureState::Closed    ? core::PluginDeviceReason_NO_HARDWARE_SIGNAL :
                                   target == CaptureState::Streaming ? core::PluginDeviceReason_NONE :
                                   target == CaptureState::Faulted   ? core::PluginDeviceReason_DEVICE_ERROR :
                                                                       core::PluginDeviceReason_NO_CURRENT_DATA));
    }
}

TEST_CASE("wrist matrix drains terminal status during blocked status or data pushes", "[wrist][unit]")
{
    for (const auto terminal : { CaptureState::Closed, CaptureState::Faulted })
        for (int pending = 0; pending < 4; ++pending)
            for (bool inflight_data : { false, true })
                for (int repeat = 0; repeat < 20; ++repeat)
                {
                    INFO("terminal=" << static_cast<int>(terminal) << " pending=" << pending
                                     << " inflight_data=" << inflight_data << " repeat=" << repeat);
                    Runtime runtime;
                    OutputConfig output;
                    output.collection_prefix = output.plugin_root_id = "/wrist/drain_matrix";
                    auto sink = make_schema_metadata_sink(output, {}, runtime.handles());
                    sink->on_capture_status(CaptureState::Streaming, "");
                    REQUIRE(runtime.wait_status(
                        output.plugin_root_id + "/device_status", core::PluginDeviceState_CONNECTED));
                    // A dispatch barrier establishes an in-flight push before close; no polling sleep guesses its
                    // position.
                    runtime.block(inflight_data ? "frame_metadata" : "plugin_device_status");
                    if (inflight_data)
                        sink->on_frame_metadata(VideoFrame{});
                    else
                        sink->on_capture_status(CaptureState::Starting, "barrier");
                    REQUIRE(runtime.wait_blocked());
                    if (pending == 1)
                        sink->on_frame_metadata(VideoFrame{});
                    else if (pending > 1)
                    {
                        CaptureBatch batch;
                        const auto sensor = pending == 2 ? ImuSensor::Accel : ImuSensor::Gyro;
                        batch.imu.push_back({ sensor, 1, { 1, 2, 3 }, 1, false, { 1, 2, 3 }, 25 });
                        sink->on_imu_batch(batch, sensor);
                    }
                    sink->on_capture_status(terminal, terminal == CaptureState::Faulted ? "matrix failure" : "");
                    std::promise<void> invoked;
                    auto started = invoked.get_future();
                    auto closing = std::async(std::launch::async,
                                              [&]
                                              {
                                                  invoked.set_value();
                                                  sink->close();
                                              });
                    started.wait();
                    CHECK(closing.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout);
                    runtime.unblock();
                    REQUIRE(closing.wait_for(std::chrono::seconds(3)) == std::future_status::ready);
                    closing.get();
                    sink->close();
                    CHECK_FALSE(runtime.block_timed_out);
                    const auto& status_bytes =
                        runtime.collections.at(output.plugin_root_id + "/device_status").samples.back().bytes;
                    const auto* device =
                        flatbuffers::GetRoot<core::PluginDeviceStatusSnapshot>(status_bytes.data())->devices()->Get(0);
                    CHECK(device->state() == (terminal == CaptureState::Closed ? core::PluginDeviceState_DISCONNECTED :
                                                                                 core::PluginDeviceState_FAILED));
                    CHECK(device->reason() == (terminal == CaptureState::Closed ?
                                                   core::PluginDeviceReason_NO_HARDWARE_SIGNAL :
                                                   core::PluginDeviceReason_DEVICE_ERROR));
                    CHECK((device->error() ? device->error()->str() : std::string{}) ==
                          (terminal == CaptureState::Faulted ? "matrix failure" : ""));
                    CHECK(runtime.collections.at(output.collection_prefix + "/Color").samples.size() ==
                          static_cast<size_t>(inflight_data) + (pending == 1));
                    CHECK(runtime.collections.at(output.collection_prefix + "/Accel").samples.size() ==
                          static_cast<size_t>(pending == 2));
                    CHECK(runtime.collections.at(output.collection_prefix + "/Gyro").samples.size() ==
                          static_cast<size_t>(pending == 3));
                    bool destroyed = false;
                    for (const auto& event : runtime.events)
                    {
                        if (event.starts_with("destroy:"))
                            destroyed = true;
                        if (event.starts_with("push:"))
                            CHECK_FALSE(destroyed);
                    }
                    for (const auto& [id, collection] : runtime.collections)
                        CHECK(collection.destroyed);
                }
}

TEST_CASE("wrist matrix cleans up individual collection creation failures", "[wrist][unit]")
{
    for (const auto& sensor : { "Color", "Accel", "Gyro" })
    {
        INFO(sensor);
        Runtime runtime;
        OutputConfig output;
        output.collection_prefix = "/wrist/create_matrix";
        runtime.fail_create_id = output.collection_prefix + "/" + sensor;
        CHECK_THROWS(make_schema_metadata_sink(output, {}, runtime.handles()));
        for (const auto& [id, collection] : runtime.collections)
            CHECK(collection.destroyed);
    }
}

TEST_CASE("wrist matrix rejects missing tensor and time entrypoints", "[wrist][unit]")
{
    for (const auto* name :
         { "xrCreatePushTensorCollectionNV", "xrPushTensorCollectionDataNV", "xrDestroyPushTensorCollectionNV",
           "xrConvertTimespecTimeToTimeKHR", "xrConvertTimeToTimespecTimeKHR" })
    {
        INFO(name);
        Runtime runtime;
        runtime.missing_function = name;
        OutputConfig output;
        output.collection_prefix = "/wrist/missing_entrypoint_matrix";
        CHECK_THROWS(make_schema_metadata_sink(output, {}, runtime.handles()));
        CHECK(runtime.collections.empty());
    }
}

TEST_CASE("wrist matrix propagates each data transport failure", "[wrist][unit]")
{
    for (const auto& tensor : { "frame_metadata", "Accel", "Gyro" })
    {
        INFO(tensor);
        Runtime runtime;
        OutputConfig output;
        output.collection_prefix = "/wrist/push_matrix";
        // IMU instances share a tensor identifier; isolate the chosen sensor by the submitted task.
        runtime.fail_push_id = std::string(tensor) == "frame_metadata" ? "frame_metadata" : "imu_batch";
        auto sink = make_schema_metadata_sink(output, {}, runtime.handles());
        if (std::string(tensor) == "frame_metadata")
            sink->on_frame_metadata(VideoFrame{});
        else
        {
            const auto sensor = std::string(tensor) == "Accel" ? ImuSensor::Accel : ImuSensor::Gyro;
            CaptureBatch batch;
            batch.imu.push_back({ sensor, 1, { 1, 2, 3 }, 1, false, { 1, 2, 3 }, 25 });
            sink->on_imu_batch(batch, sensor);
        }
        CHECK_THROWS(sink->close());
        for (const auto& [id, collection] : runtime.collections)
            CHECK(collection.destroyed);
    }
}

TEST_CASE("wrist matrix bounds the blocked data queue without silently dropping accepted samples", "[wrist][unit]")
{
    Runtime runtime;
    OutputConfig output;
    output.collection_prefix = "/wrist/queue_matrix";
    runtime.block("frame_metadata");
    auto sink = make_schema_metadata_sink(output, {}, runtime.handles());
    sink->on_frame_metadata(VideoFrame{});
    REQUIRE(runtime.wait_blocked());
    for (int count = 0; count < 4096; ++count)
        sink->on_frame_metadata(VideoFrame{});
    CHECK_THROWS_WITH(sink->on_frame_metadata(VideoFrame{}), Catch::Matchers::ContainsSubstring("queue is full"));
    runtime.unblock();
    sink->close();
    CHECK_FALSE(runtime.block_timed_out);
    CHECK(runtime.collections.at(output.collection_prefix + "/Color").samples.size() == 4097);
}

TEST_CASE("wrist matrix keeps data alive across status creation and push failures", "[wrist][unit]")
{
    for (bool fail_creation : { false, true })
    {
        INFO("fail_creation=" << fail_creation);
        Runtime runtime;
        OutputConfig output;
        output.collection_prefix = output.plugin_root_id = "/wrist/status_failure_matrix";
        if (fail_creation)
            runtime.fail_create_id = output.plugin_root_id + "/device_status";
        else
            runtime.fail_push_id = "plugin_device_status";
        auto sink = make_schema_metadata_sink(output, {}, runtime.handles());
        sink->on_capture_status(CaptureState::Streaming, "");
        sink->on_frame_metadata(VideoFrame{});
        REQUIRE(runtime.wait_for_samples(output.collection_prefix + "/Color", 1));
        {
            std::lock_guard<std::mutex> lock(runtime.mutex);
            runtime.fail_create_id.clear();
            runtime.fail_push_id.clear();
        }
        sink->on_capture_status(CaptureState::Starting, "recover status transport");
        sink->on_capture_status(CaptureState::Streaming, "");
        REQUIRE(runtime.wait_status(output.plugin_root_id + "/device_status", core::PluginDeviceState_CONNECTED));
        sink->on_frame_metadata(VideoFrame{});
        sink->close();
        CHECK(runtime.collections.at(output.collection_prefix + "/Color").samples.size() == 2);
        CHECK(sink->error().empty());
        for (const auto& [id, collection] : runtime.collections)
            CHECK(collection.destroyed);
    }
}

TEST_CASE("wrist matrix rejects oversized payloads and invalid clock inputs", "[wrist][unit]")
{
    for (int failure = 0; failure < 3; ++failure)
    {
        INFO("failure=" << failure);
        Runtime runtime;
        OutputConfig output;
        output.collection_prefix = "/wrist/invalid_input_matrix";
        auto sink = make_schema_metadata_sink(output, {}, runtime.handles());
        if (failure == 0)
        {
            CaptureBatch batch;
            for (int index = 0; index < 500; ++index)
                batch.imu.push_back(
                    { ImuSensor::Accel, static_cast<uint64_t>(index), { 1, 2, 3 }, 1, false, { 1, 2, 3 }, 25 });
            sink->on_imu_batch(batch, ImuSensor::Accel);
            CHECK_THROWS_WITH(sink->close(), Catch::Matchers::ContainsSubstring("exceeds max_flatbuffer_size"));
        }
        else
        {
            VideoFrame frame;
            if (failure == 1)
            {
                frame.time.retrieved_monotonic_ns = -1;
                sink->on_frame_metadata(frame);
                CHECK_THROWS(sink->close());
            }
            else
            {
                frame.time.device_us = std::numeric_limits<uint64_t>::max();
                CHECK_THROWS(sink->on_frame_metadata(frame));
                sink->close();
            }
        }
        for (const auto& [id, collection] : runtime.collections)
        {
            CHECK(collection.samples.empty());
            CHECK(collection.destroyed);
        }
    }
}
