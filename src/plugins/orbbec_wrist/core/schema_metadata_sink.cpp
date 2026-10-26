// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "metadata_sink.hpp"

#include <log_bridge/logger.hpp>
#include <oxr/oxr_session.hpp>
#include <plugin_utils/plugin_device_status_publisher.hpp>
#include <pusherio/schema_pusher.hpp>
#include <schema/serialized.hpp>

#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace plugins::wrist
{
namespace
{
class SchemaMetadataSink final : public IMetadataSink
{
public:
    SchemaMetadataSink(const OutputConfig& output,
                       const CaptureConfig& capture,
                       const core::OpenXRSessionHandles& handles,
                       std::shared_ptr<core::OpenXRSession> session = {})
        : capture_(capture), plugin_root_id_(output.plugin_root_id), session_(std::move(session)), handles_(handles)
    {
        if (!output.collection_prefix.empty())
        {
            video_pusher_ = pusher(output.collection_prefix + "/Color", "frame_metadata", 4096);
            if (capture.enable_imu)
            {
                imu_pushers_[0] = pusher(output.collection_prefix + "/Accel", "imu_batch", 32768);
                imu_pushers_[1] = pusher(output.collection_prefix + "/Gyro", "imu_batch", 32768);
            }
        }
        observed_ns_ = monotonic_now_ns();
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
            logger_->error("Orbbec Wrist SchemaPusher shutdown failed: {}", error.what());
        }
    }
    void on_frame_metadata(const VideoFrame& frame) override
    {
        enqueue<core::OrbbecWristFrameMetadata>(video_pusher_.get(), frame_payload(frame), frame.time);
    }
    void on_imu_batch(const CaptureBatch& batch, ImuSensor sensor) override
    {
        if (batch.imu.empty())
            return;
        enqueue<core::OrbbecWristImuBatch>(imu_pushers_[sensor == ImuSensor::Accel ? 0 : 1].get(),
                                           imu_payload(batch, sensor, capture_), batch.imu.back().time);
    }
    void on_capture_status(CaptureState state, const std::string& error) override
    {
        if (plugin_root_id_.empty())
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_)
            return;
        status_changed_ = status_changed_ || state != state_ || error != state_error_;
        state_ = state;
        state_error_ = error;
        observed_ns_ = monotonic_now_ns();
        if (status_changed_)
            wake_.notify_one();
    }
    void close() override
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        wake_.notify_all();
        if (publisher_.joinable())
            publisher_.join();
        video_pusher_.reset();
        for (auto& pusher : imu_pushers_)
            pusher.reset();
        status_publisher_.reset();
        session_.reset();
        const auto failure = error();
        if (!failure.empty())
            throw std::runtime_error("Orbbec Wrist SchemaPusher failed: " + failure);
    }
    std::string error() const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return publish_error_;
    }

private:
    std::unique_ptr<core::SchemaPusher> pusher(const std::string& collection, const std::string& tensor, size_t size)
    {
        return std::make_unique<core::SchemaPusher>(
            handles_, core::SchemaPusherConfig{ .collection_id = collection,
                                                .max_flatbuffer_size = size,
                                                .tensor_identifier = tensor,
                                                .localized_name = "Orbbec Wrist " + tensor,
                                                .app_name = "OrbbecWristCameraPlugin" });
    }
    template <class Table>
    void enqueue(core::SchemaPusher* pusher, const typename Table::NativeTableType& value, const FrameTime& time)
    {
        if (!pusher)
            return;
        const auto payload = core::pack<Table>(value);
        const auto device_ns = device_timestamp_ns(time.device_us);
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || !publish_error_.empty())
            throw std::runtime_error("Orbbec Wrist SchemaPusher is closed or failed: " + publish_error_);
        if (tasks_.size() >= kMaxQueuedEvents)
            throw std::runtime_error("Orbbec Wrist SchemaPusher queue is full; capture stopped to avoid silent loss");
        if (!queue_warning_emitted_ && tasks_.size() + 1 >= kMaxQueuedEvents * 85 / 100)
        {
            queue_warning_emitted_ = true;
            logger_->warn("Orbbec Wrist SchemaPusher queue reached 85%; capture will stop if it fills");
        }
        tasks_.push_back(
            [pusher, payload, local_ns = time.retrieved_monotonic_ns, device_ns]
            {
                const auto bytes = payload.buffer();
                pusher->push_buffer(bytes.data(), bytes.size(), local_ns, device_ns);
            });
        wake_.notify_one();
    }
    void publish_status(CaptureState state, const std::string& error, int64_t observed)
    {
        try
        {
            if (!status_publisher_)
                status_publisher_ =
                    std::make_unique<plugin_utils::PluginDeviceStatusPublisher>(handles_, plugin_root_id_);
            plugin_utils::PluginDeviceStatusEntry entry;
            entry.path = "/camera/wrist";
            entry.error = error;
            entry.state = state == CaptureState::Streaming ? core::PluginDeviceState_CONNECTED :
                          state == CaptureState::Faulted   ? core::PluginDeviceState_FAILED :
                          state == CaptureState::Closed    ? core::PluginDeviceState_DISCONNECTED :
                                                             core::PluginDeviceState_DEGRADED;
            entry.reason = state == CaptureState::Streaming ? core::PluginDeviceReason_NONE :
                           state == CaptureState::Faulted   ? core::PluginDeviceReason_DEVICE_ERROR :
                           state == CaptureState::Closed    ? core::PluginDeviceReason_NO_HARDWARE_SIGNAL :
                                                              core::PluginDeviceReason_NO_CURRENT_DATA;
            // A heartbeat does not refresh the capture owner's observation time.
            status_publisher_->publish_if_changed({ entry }, observed);
            status_warning_emitted_ = false;
        }
        catch (const std::exception& error)
        {
            if (!status_warning_emitted_)
                logger_->warn("Orbbec Wrist device status unavailable; capture continues: {}", error.what());
            status_warning_emitted_ = true;
        }
    }
    void publish_loop()
    {
        auto next_status = std::chrono::steady_clock::now();
        while (true)
        {
            std::function<void()> task;
            CaptureState state = CaptureState::Closed;
            std::string state_error;
            int64_t observed = 0;
            bool send_status = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                const auto ready = [this] { return stopping_ || !tasks_.empty() || status_changed_; };
                if (plugin_root_id_.empty())
                    wake_.wait(lock, ready);
                else
                    wake_.wait_until(lock, next_status, ready);
                // A pending terminal status must be sent even after the data queue drains.
                if (stopping_ && tasks_.empty() && !status_changed_)
                    return;
                const auto now = std::chrono::steady_clock::now();
                if (!plugin_root_id_.empty() && (status_changed_ || now >= next_status))
                {
                    state = state_;
                    state_error = state_error_;
                    observed = observed_ns_;
                    send_status = true;
                    status_changed_ = false;
                    next_status = now + std::chrono::milliseconds(250);
                }
                if (!tasks_.empty())
                {
                    task = std::move(tasks_.front());
                    tasks_.pop_front();
                }
            }
            if (send_status)
                publish_status(state, state_error, observed);
            if (!task)
                continue;
            try
            {
                task();
            }
            catch (...)
            {
                std::string failure = "Unknown Orbbec Wrist SchemaPusher failure";
                try
                {
                    throw;
                }
                catch (const std::exception& error)
                {
                    failure = error.what();
                }
                catch (...)
                {
                }
                std::lock_guard<std::mutex> lock(mutex_);
                publish_error_ = failure;
                tasks_.clear();
                logger_->error("Orbbec Wrist SchemaPusher failure: {}", failure);
            }
        }
    }
    static constexpr size_t kMaxQueuedEvents = 4096;
    CaptureConfig capture_;
    std::string plugin_root_id_;
    std::shared_ptr<core::OpenXRSession> session_;
    core::OpenXRSessionHandles handles_;
    std::unique_ptr<core::SchemaPusher> video_pusher_;
    std::array<std::unique_ptr<core::SchemaPusher>, 2> imu_pushers_;
    std::unique_ptr<plugin_utils::PluginDeviceStatusPublisher> status_publisher_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<std::function<void()>> tasks_;
    std::thread publisher_;
    CaptureState state_ = CaptureState::Closed;
    std::string state_error_;
    int64_t observed_ns_ = 0;
    bool status_changed_ = false;
    bool stopping_ = false;
    bool queue_warning_emitted_ = false;
    bool status_warning_emitted_ = false;
    std::string publish_error_;
    std::shared_ptr<spdlog::logger> logger_ =
        isaaccapture::Logger::get("isaaccapture.plugins.orbbec_wrist.SchemaMetadataSink");
};
} // namespace

std::unique_ptr<IMetadataSink> make_schema_metadata_sink(const OutputConfig& output, const CaptureConfig& capture)
{
    auto session = std::make_shared<core::OpenXRSession>(
        "OrbbecWristCameraPlugin", core::SchemaPusher::get_required_extensions(), false);
    return std::make_unique<SchemaMetadataSink>(output, capture, session->get_handles(), session);
}

std::unique_ptr<IMetadataSink> make_schema_metadata_sink(const OutputConfig& output,
                                                         const CaptureConfig& capture,
                                                         const core::OpenXRSessionHandles& handles)
{
    return std::make_unique<SchemaMetadataSink>(output, capture, handles);
}
} // namespace plugins::wrist
