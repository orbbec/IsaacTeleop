# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

set(orbbec_camera_core_sources
    orbbec_camera.cpp
    inc/orbbec_camera/orbbec_camera.hpp
)

if(ORBBEC_ENABLE_PREVIEW)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ORBBEC_PREVIEW REQUIRED IMPORTED_TARGET sdl2 libavcodec libavutil libswscale)
    list(APPEND orbbec_camera_core_sources
        preview.cpp
        preview.hpp
    )
endif()

add_library(orbbec_camera_core STATIC ${orbbec_camera_core_sources})

target_include_directories(orbbec_camera_core
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

target_link_libraries(orbbec_camera_core
    PUBLIC
        isaacteleop_schema
    PRIVATE
        ob::OrbbecSDK
        mcap::mcap_core
        oxr::oxr_core
        pusherio::pusherio
)

if(ORBBEC_ENABLE_PREVIEW)
    target_compile_definitions(orbbec_camera_core PRIVATE ORBBEC_ENABLE_PREVIEW=1)
    target_link_libraries(orbbec_camera_core PRIVATE PkgConfig::ORBBEC_PREVIEW)
endif()

add_library(orbbec::camera_core ALIAS orbbec_camera_core)
