// SPDX-FileCopyrightText: Copyright (c) 2026 ORBBEC INC. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <schema/ego_camera_generated.h>

#include <array>
#include <stdexcept>

namespace plugins::ego
{

struct CameraStreamDescriptor
{
    core::EgoCameraStream stream;
    const char* name;
    const char* timing_name;
    bool side;
};

// SDK stream labels identify logical sensors; optical directions require independent verification.
inline constexpr std::array<CameraStreamDescriptor, 4> kCameraStreams{
    { { core::EgoCameraStream_ColorLeft, "ColorLeft", "left_rgb", false },
      { core::EgoCameraStream_ColorRight, "ColorRight", "right_rgb", false },
      { core::EgoCameraStream_ColorSideLeft, "ColorSideLeft", "side_left_rgb", true },
      { core::EgoCameraStream_ColorSideRight, "ColorSideRight", "side_right_rgb", true } }
};

inline const CameraStreamDescriptor& camera_stream_descriptor(core::EgoCameraStream stream)
{
    for (const auto& descriptor : kCameraStreams)
        if (descriptor.stream == stream)
            return descriptor;
    throw std::invalid_argument("Unsupported Ego camera stream");
}

} // namespace plugins::ego
