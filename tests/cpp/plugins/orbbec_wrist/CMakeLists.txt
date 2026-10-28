# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT TARGET Teleop::plugin_utils)
    return()
endif()

find_package(Threads REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# Compile the private production sources with an SDK-free backend; no plugin API is exported.
set(wrist_source_dir "${CMAKE_SOURCE_DIR}/src/plugins/orbbec_wrist")
add_executable(wrist_camera_tests
    test_orbbec_wrist_camera.cpp
    test_capture_output.cpp
    test_frame_sink.cpp
    test_schema_metadata_sink.cpp
    "${wrist_source_dir}/core/orbbec_wrist_camera.cpp"
    "${wrist_source_dir}/core/sdk_unavailable.cpp"
    "${wrist_source_dir}/core/frame_sink.cpp"
    "${wrist_source_dir}/core/metadata_sink.cpp"
    "${wrist_source_dir}/core/schema_metadata_sink.cpp"
    "${wrist_source_dir}/core/mcap_metadata_sink.cpp"
    "${wrist_source_dir}/core/mcap_impl.cpp"
)
target_include_directories(wrist_camera_tests PRIVATE "${CMAKE_SOURCE_DIR}/src/plugins")
target_link_libraries(wrist_camera_tests
    PRIVATE
        isaacteleop_schema
        deviceio::replay_trackers
        deviceio::live_trackers
        mcap::mcap_core
        oxr::oxr_core
        pusherio::pusherio
        Teleop::plugin_utils
        log_bridge::log_bridge_core
        Threads::Threads
        nlohmann_json::nlohmann_json
        Catch2::Catch2WithMain
)
if(ORBBEC_WRIST_ENABLE_PREVIEW AND TARGET PkgConfig::ORBBEC_WRIST_PREVIEW)
    target_sources(wrist_camera_tests PRIVATE test_preview.cpp "${wrist_source_dir}/core/preview.cpp")
    target_link_libraries(wrist_camera_tests PRIVATE PkgConfig::ORBBEC_WRIST_PREVIEW)
endif()
catch_discover_tests(wrist_camera_tests ADD_TAGS_AS_LABELS)

if(TARGET orbbec_wrist_camera_plugin)
    add_test(NAME wrist_plugin_help COMMAND orbbec_wrist_camera_plugin --help)
    add_test(NAME wrist_plugin_invalid_args COMMAND orbbec_wrist_camera_plugin --serial-number= --duration=-1)
    set_tests_properties(wrist_plugin_invalid_args PROPERTIES WILL_FAIL TRUE)
    add_test(NAME wrist_plugin_space_args COMMAND orbbec_wrist_camera_plugin
        --serial-number "SN = 123" --video-output "a path/video=1.h264"
        --format h264 --duration 60 --imu-rate 400 --help)
    add_test(NAME wrist_plugin_equals_args COMMAND orbbec_wrist_camera_plugin
        "--serial-number=SN = 123" "--video-output=a path/video=1.h264"
        --format=h264 --duration=60 --imu-rate=400 --help)
    add_test(NAME wrist_plugin_missing_value COMMAND orbbec_wrist_camera_plugin --serial-number)
    add_test(NAME wrist_plugin_empty_value COMMAND orbbec_wrist_camera_plugin --serial-number= --help)
    add_test(NAME wrist_plugin_option_as_value COMMAND orbbec_wrist_camera_plugin --serial-number --help)
    add_test(NAME wrist_plugin_flag_with_value COMMAND orbbec_wrist_camera_plugin --preview=true --help)
    set_tests_properties(wrist_plugin_missing_value wrist_plugin_empty_value
        wrist_plugin_option_as_value wrist_plugin_flag_with_value PROPERTIES WILL_FAIL TRUE)
endif()
