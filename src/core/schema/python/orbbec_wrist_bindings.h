// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "schema_serialized.h"

#include <pybind11/stl.h>
#include <schema/orbbec_wrist_camera_generated.h>
#include <schema/orbbec_wrist_imu_generated.h>

namespace core
{
inline void bind_orbbec_wrist(py::module& m)
{
    py::enum_<OrbbecWristCameraStream>(m, "OrbbecWristCameraStream").value("Color", OrbbecWristCameraStream_Color);
    py::enum_<OrbbecWristPixelFormat>(m, "OrbbecWristPixelFormat")
        .value("Mjpg", OrbbecWristPixelFormat_Mjpg)
        .value("H264", OrbbecWristPixelFormat_H264)
        .value("H265", OrbbecWristPixelFormat_H265);
    serialized_class<OrbbecWristFrameMetadata>(m, "OrbbecWristFrameMetadata", "Single Color frame metadata.")
        .def_property_readonly("stream", field(&OrbbecWristFrameMetadata::stream))
        .def_property_readonly("sequence_number", field(&OrbbecWristFrameMetadata::sequence_number))
        .def_property_readonly("width", field(&OrbbecWristFrameMetadata::width))
        .def_property_readonly("height", field(&OrbbecWristFrameMetadata::height))
        .def_property_readonly("fps", field(&OrbbecWristFrameMetadata::fps))
        .def_property_readonly("pixel_format", field(&OrbbecWristFrameMetadata::pixel_format))
        .def_property_readonly("encoded_bytes", field(&OrbbecWristFrameMetadata::encoded_bytes))
        .def_property_readonly("capture_epoch", field(&OrbbecWristFrameMetadata::capture_epoch))
        .def_property_readonly("device_timestamp_us", field(&OrbbecWristFrameMetadata::device_timestamp_us))
        .def_property_readonly("system_timestamp_us", field(&OrbbecWristFrameMetadata::system_timestamp_us))
        .def_property_readonly("retrieved_monotonic_ns", field(&OrbbecWristFrameMetadata::retrieved_monotonic_ns))
        .def_property_readonly("discontinuity", field(&OrbbecWristFrameMetadata::discontinuity));
    bind_record<OrbbecWristFrameMetadataRecord, OrbbecWristFrameMetadata>(
        m, "OrbbecWristFrameMetadataRecord", "OrbbecWristFrameMetadata");

    serialized_class<OrbbecWristEncodedVideoFrame>(m, "OrbbecWristEncodedVideoFrame", "Encoded Wrist video payload.")
        .def_property_readonly("stream", field(&OrbbecWristEncodedVideoFrame::stream))
        .def_property_readonly("sequence_number", field(&OrbbecWristEncodedVideoFrame::sequence_number))
        .def_property_readonly("width", field(&OrbbecWristEncodedVideoFrame::width))
        .def_property_readonly("height", field(&OrbbecWristEncodedVideoFrame::height))
        .def_property_readonly("fps", field(&OrbbecWristEncodedVideoFrame::fps))
        .def_property_readonly("pixel_format", field(&OrbbecWristEncodedVideoFrame::pixel_format))
        .def_property_readonly("encoded_data", vector_field(&OrbbecWristEncodedVideoFrame::encoded_data))
        .def_property_readonly("capture_epoch", field(&OrbbecWristEncodedVideoFrame::capture_epoch));
    bind_record<OrbbecWristEncodedVideoFrameRecord, OrbbecWristEncodedVideoFrame>(
        m, "OrbbecWristEncodedVideoFrameRecord", "OrbbecWristEncodedVideoFrame");

    py::enum_<OrbbecWristImuSensor>(m, "OrbbecWristImuSensor")
        .value("Accel", OrbbecWristImuSensor_Accel)
        .value("Gyro", OrbbecWristImuSensor_Gyro);
    py::class_<OrbbecWristImuSample>(m, "OrbbecWristImuSample")
        .def_property_readonly("sequence_number", &OrbbecWristImuSample::sequence_number)
        .def_property_readonly("x_si", &OrbbecWristImuSample::x_si)
        .def_property_readonly("y_si", &OrbbecWristImuSample::y_si)
        .def_property_readonly("z_si", &OrbbecWristImuSample::z_si)
        .def_property_readonly("temperature_c", &OrbbecWristImuSample::temperature_c)
        .def_property_readonly("device_timestamp_us", &OrbbecWristImuSample::device_timestamp_us)
        .def_property_readonly("system_timestamp_us", &OrbbecWristImuSample::system_timestamp_us)
        .def_property_readonly("retrieved_monotonic_ns", &OrbbecWristImuSample::retrieved_monotonic_ns)
        .def_property_readonly("discontinuity", &OrbbecWristImuSample::discontinuity);
    serialized_class<OrbbecWristImuBatch>(
        m, "OrbbecWristImuBatch", "IMU samples with independent source sequence numbers and clocks.")
        .def_property_readonly("sensor", field(&OrbbecWristImuBatch::sensor))
        .def_property_readonly("sample_rate_hz", field(&OrbbecWristImuBatch::sample_rate_hz))
        .def_property_readonly("full_scale", field(&OrbbecWristImuBatch::full_scale))
        .def_property_readonly("capture_epoch", field(&OrbbecWristImuBatch::capture_epoch))
        .def_property_readonly("samples",
                               [](const Serialized<OrbbecWristImuBatch>& self)
                               {
                                   std::vector<OrbbecWristImuSample> result;
                                   if (const auto* samples = self->samples())
                                       for (const auto* sample : *samples)
                                           result.push_back(*sample);
                                   return result;
                               });
    bind_record<OrbbecWristImuBatchRecord, OrbbecWristImuBatch>(m, "OrbbecWristImuBatchRecord", "OrbbecWristImuBatch");
}
} // namespace core
