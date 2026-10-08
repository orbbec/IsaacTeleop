// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "diagnostics.hpp"
#include "metadata_sink.hpp"

#include <oxr/oxr_session.hpp>
#include <pusherio/schema_pusher.hpp>
#include <schema/ego_limits.hpp>
#include <schema/serialized.hpp>

#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace plugins::ego
{

namespace
{
constexpr size_t kMaxFlatbufferSize = core::EGO_MAX_FLATBUFFER_SIZE;
constexpr size_t kMaxQueuedEvents = 4096;
class SchemaMetadataSink final : public IMetadataSink
{
public:
    SchemaMetadataSink(const std::vector<StreamConfig>& streams, const std::string& collection_prefix)
        // TeleopSession creates DeviceIO before launching plugins; avoid an unbounded HMD wait.
        : session_(std::make_shared<core::OpenXRSession>(
              "EgoCameraPlugin", core::SchemaPusher::get_required_extensions(), false))
    {
        for (const auto& stream : streams)
        {
            const std::string name = core::EnumNameEgoCameraStream(stream.camera);
            const std::string collection_id = collection_prefix + "/" + name;
            pushers_.emplace(stream.camera, std::make_unique<core::SchemaPusher>(
                                                session_->get_handles(),
                                                core::SchemaPusherConfig{ .collection_id = collection_id,
                                                                          .max_flatbuffer_size = kMaxFlatbufferSize,
                                                                          .tensor_identifier = "frame_metadata",
                                                                          .localized_name = "Ego frame metadata",
                                                                          .app_name = "EgoCameraPlugin" }));
            detail::log_info() << "  Metadata: " << collection_id << std::endl;
        }
        imu_pushers_.emplace(
            core::EgoImuSensor_Accel, make_pusher(collection_prefix + "/Accel", "imu_batch", "Ego accelerometer"));
        imu_pushers_.emplace(
            core::EgoImuSensor_Gyro, make_pusher(collection_prefix + "/Gyro", "imu_batch", "Ego gyroscope"));
        audio_pusher_ = make_pusher(collection_prefix + "/Audio", "audio_chunk", "Ego audio index");
        calibration_pusher_ = make_pusher(collection_prefix + "/Calibration", "calibration", "Ego calibration");
        device_state_pusher_ = make_pusher(collection_prefix + "/DeviceState", "device_state", "Ego device state");
        publisher_ = std::thread([this] { publish_loop(); });
    }

    ~SchemaMetadataSink() override
    {
        try
        {
            close();
        }
        catch (const std::exception& error)
        {
            detail::log_error() << "Ego SchemaPusher shutdown failed: " << error.what() << std::endl;
        }
        catch (...)
        {
            detail::log_error() << "Ego SchemaPusher shutdown failed: unknown error" << std::endl;
        }
    }

    void on_frame_metadata(const CapturedFrame& frame) override
    {
        const auto it = pushers_.find(frame.metadata.stream);
        if (it == pushers_.end())
            return;

        enqueue<core::EgoFrameMetadata>(*it->second, frame.metadata, frame.sample_time_local_common_clock_ns,
                                        frame.sample_time_raw_device_clock_ns);
    }

    void on_imu_batch(const core::EgoImuBatchT& batch, int64_t local_ns, int64_t device_ns) override
    {
        enqueue(*imu_pushers_.at(batch.sensor), batch, local_ns, device_ns);
    }

    void on_audio_chunk(const core::EgoAudioChunkT& chunk, int64_t local_ns, int64_t device_ns) override
    {
        enqueue(*audio_pusher_, chunk, local_ns, device_ns);
    }

    void on_calibration(const core::EgoCalibrationT& calibration, int64_t local_ns, int64_t device_ns) override
    {
        enqueue(*calibration_pusher_, calibration, local_ns, device_ns);
    }

    void on_device_state(const core::EgoDeviceStateT& state, int64_t local_ns, int64_t device_ns) override
    {
        enqueue(*device_state_pusher_, state, local_ns, device_ns);
    }

    void close() override
    {
        {
            std::lock_guard<std::mutex> lock(publish_mutex_);
            stopping_ = true;
        }
        publish_wake_.notify_all();
        if (publisher_.joinable())
            publisher_.join();
        pushers_.clear();
        imu_pushers_.clear();
        audio_pusher_.reset();
        calibration_pusher_.reset();
        device_state_pusher_.reset();
        session_.reset();
        const auto failure = error();
        if (!failure.empty())
            throw std::runtime_error("Ego SchemaPusher failed: " + failure);
    }

    std::string error() const override
    {
        std::lock_guard<std::mutex> lock(publish_mutex_);
        return publish_error_;
    }

private:
    std::unique_ptr<core::SchemaPusher> make_pusher(const std::string& collection,
                                                    const std::string& tensor,
                                                    const std::string& name)
    {
        detail::log_info() << "  Metadata: " << collection << std::endl;
        return std::make_unique<core::SchemaPusher>(
            session_->get_handles(), core::SchemaPusherConfig{ .collection_id = collection,
                                                               .max_flatbuffer_size = kMaxFlatbufferSize,
                                                               .tensor_identifier = tensor,
                                                               .localized_name = name,
                                                               .app_name = "EgoCameraPlugin" });
    }

    template <typename TableT>
    void enqueue(core::SchemaPusher& pusher,
                 const typename TableT::NativeTableType& value,
                 int64_t local_ns,
                 int64_t device_ns)
    {
        const auto payload = core::pack<TableT>(value);
        enqueue_task(
            [&pusher, payload, local_ns, device_ns]()
            {
                const auto bytes = payload.buffer();
                pusher.push_buffer(bytes.data(), bytes.size(), local_ns, device_ns);
            });
    }

    void enqueue(core::SchemaPusher& pusher, const core::EgoImuBatchT& value, int64_t local_ns, int64_t device_ns)
    {
        enqueue<core::EgoImuBatch>(pusher, value, local_ns, device_ns);
    }
    void enqueue(core::SchemaPusher& pusher, const core::EgoAudioChunkT& value, int64_t local_ns, int64_t device_ns)
    {
        enqueue<core::EgoAudioChunk>(pusher, value, local_ns, device_ns);
    }
    void enqueue(core::SchemaPusher& pusher, const core::EgoCalibrationT& value, int64_t local_ns, int64_t device_ns)
    {
        enqueue<core::EgoCalibration>(pusher, value, local_ns, device_ns);
    }
    void enqueue(core::SchemaPusher& pusher, const core::EgoDeviceStateT& value, int64_t local_ns, int64_t device_ns)
    {
        enqueue<core::EgoDeviceState>(pusher, value, local_ns, device_ns);
    }

    void enqueue_task(std::function<void()> task)
    {
        std::lock_guard<std::mutex> lock(publish_mutex_);
        if (stopping_)
            throw std::runtime_error("Ego SchemaPusher is already closed");
        if (!publish_error_.empty())
            throw std::runtime_error("Ego SchemaPusher failed: " + publish_error_);
        if (tasks_.size() >= kMaxQueuedEvents)
            throw std::runtime_error("Ego SchemaPusher queue is full; capture stopped to avoid silent loss");
        if (tasks_.size() + 1 >= kMaxQueuedEvents * 85 / 100 && !warning_emitted_)
        {
            warning_emitted_ = true;
            detail::log_warning()
                << "Warning: Ego SchemaPusher queue reached 85%; capture will stop if it fills." << std::endl;
        }
        tasks_.push_back(std::move(task));
        publish_wake_.notify_one();
    }

    void publish_loop()
    {
        while (true)
        {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(publish_mutex_);
                publish_wake_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
                if (tasks_.empty())
                {
                    if (stopping_)
                        return;
                    continue;
                }
                task = std::move(tasks_.front());
                tasks_.pop_front();
            }
            try
            {
                task();
            }
            catch (const std::exception& error)
            {
                std::lock_guard<std::mutex> lock(publish_mutex_);
                publish_error_ = error.what();
                tasks_.clear();
                detail::log_error() << "Ego SchemaPusher failure: " << publish_error_ << std::endl;
            }
        }
    }

    std::shared_ptr<core::OpenXRSession> session_;
    std::map<core::EgoCameraStream, std::unique_ptr<core::SchemaPusher>> pushers_;
    std::map<core::EgoImuSensor, std::unique_ptr<core::SchemaPusher>> imu_pushers_;
    std::unique_ptr<core::SchemaPusher> audio_pusher_;
    std::unique_ptr<core::SchemaPusher> calibration_pusher_;
    std::unique_ptr<core::SchemaPusher> device_state_pusher_;
    mutable std::mutex publish_mutex_;
    std::condition_variable publish_wake_;
    std::deque<std::function<void()>> tasks_;
    std::thread publisher_;
    bool stopping_ = false;
    bool warning_emitted_ = false;
    std::string publish_error_;
};

}

std::unique_ptr<IMetadataSink> make_schema_metadata_sink(const std::vector<StreamConfig>& streams,
                                                         const std::string& prefix)
{
    return std::make_unique<SchemaMetadataSink>(streams, prefix);
}

} // namespace plugins::ego
