<!--
SPDX-FileCopyrightText: Copyright (c) 2026 ORBBEC INC. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# EGO plugin development

- Keep EGO stereo capture supported on the EGOPRO branch. Preserve stereo
  defaults, public stream identities and recorded-data compatibility; validate
  stereo whenever shared capture code changes.
- Before editing a path, read every applicable ancestor README and AGENTS.md,
  including the repository root and this directory.
- Adapt this plugin to NVIDIA's existing interfaces and lifecycle. Keep
  official source changes minimal and necessary; do not edit official README
  or AGENTS.md files as part of this adaptation.
- Use SDK stream identities for the four RGB streams. Physical lens roles,
  calibration availability and exposure synchronization require evidence;
  missing calibration must remain explicitly unavailable.
