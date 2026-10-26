// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>
#include <deviceio_trackers/orbbec_wrist_frame_metadata_tracker.hpp>
#include <deviceio_trackers/orbbec_wrist_imu_tracker.hpp>
#include <live_trackers/live_deviceio_factory.hpp>
#include <live_trackers/schema_tracker_base.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

TEST_CASE("Wrist generated trackers resolve only tensor and time extensions", "[unit][live_trackers][wrist]")
{
    std::vector<std::shared_ptr<core::ITracker>> trackers = {
        std::make_shared<core::OrbbecWristFrameMetadataTracker>("wrist/Color"),
        std::make_shared<core::OrbbecWristImuTracker>("wrist/Accel"),
        std::make_shared<core::OrbbecWristImuTracker>("wrist/Gyro"),
    };
    const auto expected = core::SchemaTrackerBase::get_required_extensions();
    const std::set<std::string> expected_set(expected.begin(), expected.end());
    for (const auto& tracker : trackers)
    {
        const auto extensions = core::LiveDeviceIOFactory::get_required_extensions({ tracker });
        CHECK(std::set<std::string>(extensions.begin(), extensions.end()) == expected_set);
    }
    const auto extensions = core::LiveDeviceIOFactory::get_required_extensions(trackers);
    CHECK(std::set<std::string>(extensions.begin(), extensions.end()) == expected_set);
}
