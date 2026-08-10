# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Regression coverage for the supplied Orbbec Ego viewer configuration."""

from pathlib import Path

import yaml


def test_orbbec_ego_display_layout_uses_the_runtime_schema() -> None:
    config_path = Path(__file__).parents[1] / "configs" / "orbbec_ego.yaml"
    config = yaml.safe_load(config_path.read_text())

    assert config["display"]["mode"] == "window"
    placement = config["display"]["placements"]["orbbec_ego"]
    assert placement["position"] == [0.0, 0.0, -1.5]
    assert placement["size"] == [1.0, 0.8125]
    assert "mode" not in config
    assert "placements" not in config
