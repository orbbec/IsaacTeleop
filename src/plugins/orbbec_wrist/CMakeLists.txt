# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(BUILD_PLUGIN_ORBBEC_WRIST_CAMERA)
        message(FATAL_ERROR "The Orbbec Wrist backend currently requires Linux SDK packages and CLOCK_MONOTONIC")
    endif()
    return()
endif()

option(ORBBEC_WRIST_ENABLE_PREVIEW "Build the Orbbec Wrist SDL2/FFmpeg preview" OFF)
if(ORBBEC_WRIST_ENABLE_PREVIEW)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ORBBEC_WRIST_PREVIEW REQUIRED IMPORTED_TARGET GLOBAL sdl2 libavcodec libavutil libswscale)
endif()

if(BUILD_PLUGIN_ORBBEC_WRIST_CAMERA)
    if(NOT DEFINED ORBBEC_SDK_ROOT AND DEFINED ENV{ORBBEC_SDK_ROOT})
        set(ORBBEC_SDK_ROOT "$ENV{ORBBEC_SDK_ROOT}" CACHE PATH "Path to external Orbbec SDK v2")
    endif()
    if(NOT ORBBEC_SDK_ROOT OR NOT EXISTS "${ORBBEC_SDK_ROOT}/lib/OrbbecSDKConfig.cmake")
        message(FATAL_ERROR "BUILD_PLUGIN_ORBBEC_WRIST_CAMERA=ON requires ORBBEC_SDK_ROOT with lib/OrbbecSDKConfig.cmake")
    endif()
    if(TARGET ob::OrbbecSDK)
        get_target_property(_wrist_sdk_includes ob::OrbbecSDK INTERFACE_INCLUDE_DIRECTORIES)
        file(REAL_PATH "${ORBBEC_SDK_ROOT}/include" _wrist_expected_include)
        file(REAL_PATH "${_wrist_sdk_includes}" _wrist_actual_include)
        if(NOT _wrist_actual_include STREQUAL _wrist_expected_include)
            message(FATAL_ERROR "Orbbec Wrist and EGO must use the same ORBBEC_SDK_ROOT in one build")
        endif()
    else()
        set(OrbbecSDK_DIR "${ORBBEC_SDK_ROOT}/lib" CACHE PATH "Selected Orbbec SDK CMake package" FORCE)
        find_package(OrbbecSDK CONFIG REQUIRED PATHS "${ORBBEC_SDK_ROOT}/lib" NO_DEFAULT_PATH)
    endif()
endif()

if(NOT BUILD_PLUGIN_ORBBEC_WRIST_CAMERA)
    return()
endif()

find_package(Threads REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(orbbec_wrist_camera_plugin
    main.cpp
    core/orbbec_wrist_camera.cpp
    core/orbbec_backend.cpp
    core/frame_sink.cpp
    core/metadata_sink.cpp
    core/schema_metadata_sink.cpp
    core/mcap_metadata_sink.cpp
    core/mcap_impl.cpp
)
target_link_libraries(orbbec_wrist_camera_plugin
    PRIVATE
        ob::OrbbecSDK
        isaacteleop_schema
        mcap::mcap_core
        oxr::oxr_core
        pusherio::pusherio
        Teleop::plugin_utils
        log_bridge::log_bridge_core
        Threads::Threads
        nlohmann_json::nlohmann_json
)
set_target_properties(orbbec_wrist_camera_plugin PROPERTIES BUILD_RPATH "${ORBBEC_SDK_ROOT}/lib")
if(ORBBEC_WRIST_ENABLE_PREVIEW)
    target_sources(orbbec_wrist_camera_plugin PRIVATE core/preview.cpp)
    target_compile_definitions(orbbec_wrist_camera_plugin PRIVATE ORBBEC_WRIST_ENABLE_PREVIEW=1)
    target_link_libraries(orbbec_wrist_camera_plugin PRIVATE PkgConfig::ORBBEC_WRIST_PREVIEW)
endif()
configure_file(plugin.yaml plugin.yaml COPYONLY)
install(TARGETS orbbec_wrist_camera_plugin RUNTIME DESTINATION plugins/orbbec_wrist COMPONENT orbbec_wrist)
install(FILES plugin.yaml README.md orbbec_wrist.example.toml DESTINATION plugins/orbbec_wrist COMPONENT orbbec_wrist)
install(PROGRAMS install.sh orbbec_wrist.sh DESTINATION plugins/orbbec_wrist COMPONENT orbbec_wrist)
install(FILES tools/verify_recording.py DESTINATION plugins/orbbec_wrist/tools COMPONENT orbbec_wrist)
