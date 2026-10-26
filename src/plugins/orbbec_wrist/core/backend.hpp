// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "orbbec_wrist_camera.hpp"

namespace plugins::wrist
{

// Injection seam for exercising the production lifecycle without SDK or hardware.
// poll() returns owned values and performs bounded waits; close() releases partial opens/starts.
class CaptureBackend
{
public:
    virtual ~CaptureBackend() = default;
    virtual std::vector<DeviceInfo> enumerate() = 0;
    virtual DeviceInfo open(const CaptureConfig& config) = 0;
    virtual void start_imu() = 0;
    virtual void start_video() = 0;
    virtual CaptureBatch poll() = 0;
    virtual void stop() = 0;
    virtual void close() = 0;
};

} // namespace plugins::wrist
