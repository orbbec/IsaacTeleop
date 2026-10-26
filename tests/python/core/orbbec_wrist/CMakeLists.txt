# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_test(NAME wrist_recording_python
    COMMAND uv run --extra dev pytest test_recording.py
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(wrist_recording_python PROPERTIES LABELS "wrist;unit")

add_test(NAME wrist_embedded_session_python
    COMMAND uv run --extra dev pytest test_embedded_session.py
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(wrist_embedded_session_python PROPERTIES LABELS "wrist;unit")
