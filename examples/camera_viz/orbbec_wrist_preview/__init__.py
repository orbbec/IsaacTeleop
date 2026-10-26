# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""SDK capture binding for the Orbbec Wrist live-preview source."""

try:
    from ._orbbec_wrist_capture import Capture
except ImportError as exc:
    raise ImportError(
        "Orbbec Wrist camera_viz binding is not built. Run "
        "`orbbec_wrist_preview/build.sh --sdk-root PATH`."
    ) from exc

__all__ = ["Capture"]
