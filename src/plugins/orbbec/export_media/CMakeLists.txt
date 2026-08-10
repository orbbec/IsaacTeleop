# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_executable(orbbec_mcap_export_media
    export_embedded_media.cpp
)

target_link_libraries(orbbec_mcap_export_media
    PRIVATE
        orbbec::camera_core
        mcap::mcap_core
)

set_target_properties(orbbec_mcap_export_media PROPERTIES
    BUILD_RPATH "${ORBBEC_SDK_ROOT}/lib"
    INSTALL_RPATH "$ORIGIN"
)

add_custom_command(TARGET orbbec_mcap_export_media POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/src/plugins/orbbec_camera"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:orbbec_mcap_export_media>"
            "${CMAKE_BINARY_DIR}/src/plugins/orbbec_camera/orbbec_mcap_export_media"
    COMMENT "Preparing Orbbec embedded-MCAP exporter"
)

install(TARGETS orbbec_mcap_export_media RUNTIME DESTINATION plugins/orbbec_camera)
