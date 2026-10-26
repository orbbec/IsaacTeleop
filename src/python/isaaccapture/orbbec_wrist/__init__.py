# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Embedded Orbbec Wrist recording and multi-device MCAP finalization."""

from .merge import merge_recording
from .recording import embedded_recording

__all__ = ["embedded_recording", "merge_recording"]
