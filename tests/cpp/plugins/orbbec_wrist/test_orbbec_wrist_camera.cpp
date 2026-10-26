// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>
#include <orbbec_wrist/core/backend.hpp>

#include <chrono>
#include <limits>
#include <memory>
#include <thread>

using namespace plugins::wrist;
namespace
{
class FakeBackend : public CaptureBackend
{
public:
    CaptureConfig config;
    DeviceInfo info{
        "fake-test-device", "fake-port", "fake Wrist", "test-fw", 0x2bc5, 0x1203, { PixelFormat::H264, 1600, 1200, 30 }
    };
    std::vector<std::string> calls;
    bool video = false, imu = false, fail_open = false, fail_video = false, fail_close = false;
    bool disconnected = false, malformed = false, silent = false, backwards = false;
    uint64_t sequence = 1;
    std::vector<DeviceInfo> enumerate() override
    {
        return { info };
    }
    DeviceInfo open(const CaptureConfig& value) override
    {
        calls.push_back("open");
        config = value;
        if (fail_open)
            throw CaptureError(ErrorCode::DeviceNotFound, "absent");
        if (info.serial_number.empty())
            info = { config.serial_number,
                     "fake-port",
                     "fake Wrist",
                     "test-fw",
                     0x2bc5,
                     0x1203,
                     { PixelFormat::H264, 1600, 1200, 30 } };
        return info;
    }
    void start_imu() override
    {
        calls.push_back("imu");
        imu = true;
    }
    void start_video() override
    {
        calls.push_back("video");
        video = true;
        if (fail_video)
            throw CaptureError(ErrorCode::VendorError, "partial start");
    }
    CaptureBatch poll() override
    {
        if (disconnected)
        {
            disconnected = false;
            throw CaptureError(ErrorCode::Disconnected, "unplugged");
        }
        if (silent)
            return {};
        const auto seq = backwards ? 0 : sequence++;
        const FrameTime time{ seq * 1000, seq * 1000 + 10, monotonic_now_ns() };
        CaptureBatch batch;
        if (video)
            batch.video.push_back({ info.video, seq, time, 0, false,
                                    malformed ? std::vector<uint8_t>{} : std::vector<uint8_t>{ 1, 2, 3 } });
        if (imu)
        {
            batch.imu.push_back({ ImuSensor::Accel, seq, time, 0, false, { 1, 2, 3 }, 23 });
            batch.imu.push_back({ ImuSensor::Gyro, seq, time, 0, false, { 4, 5, 6 }, 24 });
        }
        return batch;
    }
    void stop() override
    {
        calls.push_back("stop");
        imu = video = false;
    }
    void close() override
    {
        calls.push_back("close");
        stop();
        if (fail_close)
            throw CaptureError(ErrorCode::CleanupFailed, "cleanup failed");
    }
};
CaptureConfig config()
{
    CaptureConfig value;
    value.serial_number = "fake-test-device";
    value.startup_timeout_ms = 1000;
    value.reconnect_interval_ms = 1;
    return value;
}
struct Fixture
{
    FakeBackend* fake;
    WristCamera camera;
    Fixture() : Fixture(std::make_unique<FakeBackend>())
    {
    }
    explicit Fixture(std::unique_ptr<FakeBackend> backend) : fake(backend.get()), camera(std::move(backend))
    {
    }
};
} // namespace

TEST_CASE("wrist validates config before hardware access", "[wrist][unit]")
{
    Fixture f;
    auto value = config();
    value.video.width = 640;
    REQUIRE_THROWS_AS(f.camera.open(value), CaptureError);
    REQUIRE(f.fake->calls.empty());
    value = config();
    value.imu_rate_hz = 17;
    REQUIRE_THROWS_AS(f.camera.open(value), CaptureError);
    value = config();
    value.video.format = static_cast<PixelFormat>(99);
    REQUIRE_THROWS_AS(f.camera.open(value), CaptureError);
}

TEST_CASE("wrist starts IMU before Color and supports repeated lifecycle", "[wrist][unit]")
{
    Fixture f;
    REQUIRE_THROWS_AS(f.camera.start(), CaptureError);
    f.camera.open(config());
    f.camera.start();
    REQUIRE(f.fake->calls == std::vector<std::string>{ "open", "imu", "video" });
    f.camera.start();
    auto batch = f.camera.update();
    REQUIRE(batch.video.size() == 1);
    REQUIRE(batch.imu.size() == 2);
    REQUIRE(batch.video[0].capture_epoch == 1);
    f.camera.stop();
    f.camera.stop();
    f.camera.start();
    REQUIRE(f.camera.update().video[0].capture_epoch == 2);
    f.camera.close();
    f.camera.close();
    REQUIRE(f.camera.state() == CaptureState::Closed);
    REQUIRE(batch.video[0].encoded_data == std::vector<uint8_t>{ 1, 2, 3 });
    f.camera.open(config());
    f.camera.start();
    REQUIRE(f.camera.update().video[0].capture_epoch == 3);
}

TEST_CASE("wrist failed opens and partial starts release all resources", "[wrist][unit]")
{
    Fixture f;
    SECTION("open")
    {
        f.fake->fail_open = true;
        REQUIRE_THROWS_AS(f.camera.open(config()), CaptureError);
        REQUIRE(f.camera.state() == CaptureState::Closed);
        REQUIRE(f.fake->calls.back() == "stop");
        f.fake->fail_open = false;
        f.camera.open(config());
    }
    SECTION("video after IMU")
    {
        f.camera.open(config());
        f.fake->fail_video = true;
        REQUIRE_THROWS_AS(f.camera.start(), CaptureError);
        REQUIRE_FALSE(f.fake->imu);
        REQUIRE_FALSE(f.fake->video);
        REQUIRE(f.camera.state() == CaptureState::Faulted);
        REQUIRE_THROWS_AS(f.camera.start(), CaptureError);
    }
    f.camera.close();
    f.camera.close();
}

TEST_CASE("wrist readiness timeout and cancellation clean up", "[wrist][unit]")
{
    Fixture f;
    auto value = config();
    SECTION("timeout")
    {
        value.startup_timeout_ms = 2;
        f.camera.open(value);
        f.fake->silent = true;
        REQUIRE_THROWS_AS(f.camera.start(), CaptureError);
        REQUIRE(f.camera.state() == CaptureState::Faulted);
        REQUIRE_FALSE(f.fake->imu);
    }
    SECTION("cancel")
    {
        value.stop_requested = [] { return true; };
        REQUIRE_THROWS_AS(f.camera.open(value), CaptureError);
        REQUIRE(f.camera.state() == CaptureState::Closed);
    }
}

TEST_CASE("wrist reconnect requires original identity and starts a new epoch", "[wrist][unit]")
{
    Fixture f;
    f.camera.open(config());
    f.camera.start();
    f.fake->disconnected = true;
    REQUIRE(f.camera.update().video.empty());
    REQUIRE(f.camera.state() == CaptureState::Recovering);
    SECTION("same device")
    {
        f.camera.update();
        REQUIRE(f.camera.state() == CaptureState::Streaming);
        REQUIRE(f.camera.update().video[0].capture_epoch == 2);
    }
    SECTION("changed firmware")
    {
        f.fake->info.firmware = "different";
        REQUIRE_THROWS_AS(f.camera.update(), CaptureError);
        REQUIRE(f.camera.state() == CaptureState::Faulted);
    }
    SECTION("different serial")
    {
        f.fake->info.serial_number = "another-fake";
        f.camera.update();
        REQUIRE(f.camera.state() == CaptureState::Recovering);
        REQUIRE_FALSE(f.fake->video);
    }
}

TEST_CASE("wrist disabled recovery and cleanup failure do not restart capture", "[wrist][unit]")
{
    Fixture f;
    auto value = config();
    SECTION("disabled")
    {
        value.reconnect_timeout_ms = 0;
    }
    SECTION("cleanup failed")
    {
        f.fake->fail_close = true;
    }
    f.camera.open(value);
    f.camera.start();
    f.fake->disconnected = true;
    REQUIRE_THROWS_AS(f.camera.update(), CaptureError);
    REQUIRE(f.camera.state() == CaptureState::Faulted);
    REQUIRE_FALSE(f.camera.last_error().empty());
}

TEST_CASE("wrist malformed frames fail and clock discontinuities remain visible", "[wrist][unit]")
{
    Fixture f;
    f.camera.open(config());
    f.camera.start();
    SECTION("malformed")
    {
        f.fake->malformed = true;
        REQUIRE_THROWS_AS(f.camera.update(), CaptureError);
        REQUIRE(f.camera.state() == CaptureState::Faulted);
    }
    SECTION("clock rollback")
    {
        f.camera.update();
        f.fake->backwards = true;
        const auto batch = f.camera.update();
        REQUIRE(batch.video[0].discontinuity);
        REQUIRE(batch.imu[0].discontinuity);
        REQUIRE(batch.video[0].time.device_us == 0);
    }
}

TEST_CASE("wrist timestamp conversion checks overflow", "[wrist][unit]")
{
    const auto limit = static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) / 1000;
    REQUIRE(device_timestamp_ns(123) == 123000);
    REQUIRE_NOTHROW(device_timestamp_ns(limit));
    REQUIRE_THROWS_AS(device_timestamp_ns(limit + 1), CaptureError);
}

TEST_CASE("wrist liveness and reconnect deadlines are bounded", "[wrist][unit]")
{
    Fixture f;
    auto value = config();
    value.stall_timeout_ms = 1;
    value.reconnect_timeout_ms = 2;
    f.camera.open(value);
    f.camera.start();
    f.fake->silent = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    f.camera.update();
    REQUIRE(f.camera.state() == CaptureState::Recovering);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    REQUIRE_THROWS_AS(f.camera.update(), CaptureError);
    REQUIRE(f.camera.state() == CaptureState::Faulted);
    f.camera.stop();
    REQUIRE(f.camera.state() == CaptureState::Faulted);
    f.camera.close();
}

TEST_CASE("wrist video-only mode does not request IMU", "[wrist][unit]")
{
    Fixture f;
    auto value = config();
    value.enable_imu = false;
    f.camera.open(value);
    f.camera.start();
    REQUIRE(f.fake->calls == std::vector<std::string>{ "open", "video" });
    REQUIRE(f.camera.update().imu.empty());
}

TEST_CASE("wrist selects only compatible devices and rejects ambiguous discovery", "[wrist][unit]")
{
    DeviceInfo left{ "left", "usb-left", "WristCam-L", "1.1.8", 0x2bc5, 0x1203, {} };
    auto right = left;
    right.serial_number = "right";
    auto other = left;
    other.serial_number = "ego";
    other.pid = 0x1201;
    REQUIRE(select_wrist_device({ other, left }, "").serial_number == "left");
    REQUIRE(select_wrist_device({ left, right, other }, "right").serial_number == "right");
    REQUIRE_THROWS_AS(select_wrist_device({ left, right }, ""), CaptureError);
    REQUIRE_THROWS_AS(select_wrist_device({ other }, ""), CaptureError);
    REQUIRE_THROWS_AS(select_wrist_device({ other }, "ego"), CaptureError);
    REQUIRE_THROWS_AS(select_wrist_device({ left }, "absent"), CaptureError);
}

TEST_CASE("wrist automatic selection forwards both supported IMU rates", "[wrist][unit]")
{
    for (const auto rate : { 400U, 1000U })
    {
        Fixture f;
        auto value = config();
        value.serial_number.clear();
        value.imu_rate_hz = rate;
        f.camera.open(value);
        REQUIRE(f.fake->config.serial_number == "fake-test-device");
        REQUIRE(f.fake->config.imu_rate_hz == rate);
        REQUIRE(f.fake->config.video.width == 1600);
        REQUIRE(f.fake->config.video.height == 1200);
        f.camera.start();
        REQUIRE(f.camera.update().imu.size() == 2);
    }
}
