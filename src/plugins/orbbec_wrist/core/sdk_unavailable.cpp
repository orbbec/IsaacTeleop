// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "backend.hpp"

namespace plugins::wrist
{
std::unique_ptr<CaptureBackend> make_orbbec_backend()
{
    throw CaptureError(ErrorCode::InvalidConfig, "Orbbec Wrist SDK backend requires BUILD_PLUGIN_ORBBEC_WRIST_CAMERA=ON");
}
} // namespace plugins::wrist
