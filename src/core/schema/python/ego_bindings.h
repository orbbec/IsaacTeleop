// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "schema_serialized.h"

#include <pybind11/stl.h>
#include <schema/ego_audio_generated.h>
#include <schema/ego_calibration_generated.h>
#include <schema/ego_device_state_generated.h>
#include <schema/ego_imu_generated.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace core
{

inline void bind_ego(py::module& m)
{

    py::enum_<EgoImuSensor>(m, "EgoImuSensor").value("Accel", EgoImuSensor_Accel).value("Gyro", EgoImuSensor_Gyro);

    py::class_<EgoImuSample>(m, "EgoImuSample")
        .def(py::init<double, double, double, double, int64_t, int64_t>(), py::arg("x_si") = 0.0, py::arg("y_si") = 0.0,
             py::arg("z_si") = 0.0, py::arg("temperature_c") = 0.0, py::arg("sample_time_local_common_clock_ns") = 0,
             py::arg("sample_time_raw_device_clock_ns") = 0)
        .def_property("x_si", &EgoImuSample::x_si, &EgoImuSample::mutate_x_si)
        .def_property("y_si", &EgoImuSample::y_si, &EgoImuSample::mutate_y_si)
        .def_property("z_si", &EgoImuSample::z_si, &EgoImuSample::mutate_z_si)
        .def_property("temperature_c", &EgoImuSample::temperature_c, &EgoImuSample::mutate_temperature_c)
        .def_property("sample_time_local_common_clock_ns", &EgoImuSample::sample_time_local_common_clock_ns,
                      &EgoImuSample::mutate_sample_time_local_common_clock_ns)
        .def_property("sample_time_raw_device_clock_ns", &EgoImuSample::sample_time_raw_device_clock_ns,
                      &EgoImuSample::mutate_sample_time_raw_device_clock_ns);

    serialized_class<EgoImuBatch>(m, "EgoImuBatch", "Encoded EGO payload.")
        .def(py::init(
                 [](EgoImuSensor sensor, uint64_t sequence_number, uint32_t sample_rate_hz, float full_scale,
                    const std::vector<EgoImuSample>& samples, uint32_t capture_epoch)
                 {
                     EgoImuBatchT native;
                     native.sensor = sensor;
                     native.sequence_number = sequence_number;
                     native.sample_rate_hz = sample_rate_hz;
                     native.full_scale = full_scale;
                     native.samples = samples;
                     native.capture_epoch = capture_epoch;
                     return pack<EgoImuBatch>(native);
                 }),
             py::arg("sensor") = EgoImuSensor_Accel, py::arg("sequence_number") = 0, py::arg("sample_rate_hz") = 0,
             py::arg("full_scale") = 0.0f, py::arg("samples") = std::vector<EgoImuSample>{}, py::arg("capture_epoch") = 0)
        .def_property_readonly("sensor", field(&EgoImuBatch::sensor))
        .def_property_readonly("sequence_number", field(&EgoImuBatch::sequence_number))
        .def_property_readonly("sample_rate_hz", field(&EgoImuBatch::sample_rate_hz))
        .def_property_readonly("full_scale", field(&EgoImuBatch::full_scale))
        .def_property_readonly("samples",
                               [](const Serialized<EgoImuBatch>& self)
                               {
                                   std::vector<EgoImuSample> values;
                                   const auto* encoded = self->samples();
                                   if (encoded != nullptr)
                                   {
                                       values.reserve(encoded->size());
                                       for (const auto* value : *encoded)
                                       {
                                           values.push_back(*value);
                                       }
                                   }
                                   return values;
                               })
        .def_property_readonly("capture_epoch", field(&EgoImuBatch::capture_epoch));

    bind_record<EgoImuBatchRecord, EgoImuBatch>(m, "EgoImuBatchRecord", "EgoImuBatch");

    py::enum_<EgoAudioSampleFormat>(m, "EgoAudioSampleFormat").value("S16LE", EgoAudioSampleFormat_S16LE);

    serialized_class<EgoAudioChunk>(m, "EgoAudioChunk", "Encoded EGO payload.")
        .def(py::init(
                 [](uint64_t sequence_number, uint32_t sample_rate_hz, uint16_t channel_count, uint16_t bits_per_sample,
                    EgoAudioSampleFormat sample_format, uint32_t sample_count, uint64_t wav_data_offset,
                    uint32_t byte_count, uint32_t capture_epoch)
                 {
                     EgoAudioChunkT native;
                     native.sequence_number = sequence_number;
                     native.sample_rate_hz = sample_rate_hz;
                     native.channel_count = channel_count;
                     native.bits_per_sample = bits_per_sample;
                     native.sample_format = sample_format;
                     native.sample_count = sample_count;
                     native.wav_data_offset = wav_data_offset;
                     native.byte_count = byte_count;
                     native.capture_epoch = capture_epoch;
                     return pack<EgoAudioChunk>(native);
                 }),
             py::arg("sequence_number") = 0, py::arg("sample_rate_hz") = 0, py::arg("channel_count") = 0,
             py::arg("bits_per_sample") = 0, py::arg("sample_format") = EgoAudioSampleFormat_S16LE,
             py::arg("sample_count") = 0, py::arg("wav_data_offset") = 0, py::arg("byte_count") = 0,
             py::arg("capture_epoch") = 0)
        .def_property_readonly("sequence_number", field(&EgoAudioChunk::sequence_number))
        .def_property_readonly("sample_rate_hz", field(&EgoAudioChunk::sample_rate_hz))
        .def_property_readonly("channel_count", field(&EgoAudioChunk::channel_count))
        .def_property_readonly("bits_per_sample", field(&EgoAudioChunk::bits_per_sample))
        .def_property_readonly("sample_format", field(&EgoAudioChunk::sample_format))
        .def_property_readonly("sample_count", field(&EgoAudioChunk::sample_count))
        .def_property_readonly("wav_data_offset", field(&EgoAudioChunk::wav_data_offset))
        .def_property_readonly("byte_count", field(&EgoAudioChunk::byte_count))
        .def_property_readonly("capture_epoch", field(&EgoAudioChunk::capture_epoch));

    bind_record<EgoAudioChunkRecord, EgoAudioChunk>(m, "EgoAudioChunkRecord", "EgoAudioChunk");

    serialized_class<EgoPcmAudioChunk>(m, "EgoPcmAudioChunk", "Encoded EGO payload.")
        .def(py::init(
                 [](uint64_t sequence_number, uint32_t sample_rate_hz, uint16_t channel_count, uint16_t bits_per_sample,
                    EgoAudioSampleFormat sample_format, uint32_t sample_count, const std::vector<uint8_t>& pcm_data,
                    uint32_t capture_epoch)
                 {
                     EgoPcmAudioChunkT native;
                     native.sequence_number = sequence_number;
                     native.sample_rate_hz = sample_rate_hz;
                     native.channel_count = channel_count;
                     native.bits_per_sample = bits_per_sample;
                     native.sample_format = sample_format;
                     native.sample_count = sample_count;
                     native.pcm_data = pcm_data;
                     native.capture_epoch = capture_epoch;
                     return pack<EgoPcmAudioChunk>(native);
                 }),
             py::arg("sequence_number") = 0, py::arg("sample_rate_hz") = 0, py::arg("channel_count") = 0,
             py::arg("bits_per_sample") = 0, py::arg("sample_format") = EgoAudioSampleFormat_S16LE,
             py::arg("sample_count") = 0, py::arg("pcm_data") = std::vector<uint8_t>{}, py::arg("capture_epoch") = 0)
        .def_property_readonly("sequence_number", field(&EgoPcmAudioChunk::sequence_number))
        .def_property_readonly("sample_rate_hz", field(&EgoPcmAudioChunk::sample_rate_hz))
        .def_property_readonly("channel_count", field(&EgoPcmAudioChunk::channel_count))
        .def_property_readonly("bits_per_sample", field(&EgoPcmAudioChunk::bits_per_sample))
        .def_property_readonly("sample_format", field(&EgoPcmAudioChunk::sample_format))
        .def_property_readonly("sample_count", field(&EgoPcmAudioChunk::sample_count))
        .def_property_readonly("pcm_data", vector_field(&EgoPcmAudioChunk::pcm_data))
        .def_property_readonly("capture_epoch", field(&EgoPcmAudioChunk::capture_epoch));

    bind_record<EgoPcmAudioChunkRecord, EgoPcmAudioChunk>(m, "EgoPcmAudioChunkRecord", "EgoPcmAudioChunk");

    serialized_class<EgoCameraIntrinsics>(m, "EgoCameraIntrinsics", "Encoded EGO payload.")
        .def(py::init(
                 [](uint32_t width, uint32_t height, float fx, float fy, float cx, float cy, int32_t distortion_model,
                    const std::vector<float>& distortion)
                 {
                     EgoCameraIntrinsicsT native;
                     native.width = width;
                     native.height = height;
                     native.fx = fx;
                     native.fy = fy;
                     native.cx = cx;
                     native.cy = cy;
                     native.distortion_model = distortion_model;
                     native.distortion = distortion;
                     return pack<EgoCameraIntrinsics>(native);
                 }),
             py::arg("width") = 0, py::arg("height") = 0, py::arg("fx") = 0.0f, py::arg("fy") = 0.0f,
             py::arg("cx") = 0.0f, py::arg("cy") = 0.0f, py::arg("distortion_model") = 0,
             py::arg("distortion") = std::vector<float>{})
        .def_property_readonly("width", field(&EgoCameraIntrinsics::width))
        .def_property_readonly("height", field(&EgoCameraIntrinsics::height))
        .def_property_readonly("fx", field(&EgoCameraIntrinsics::fx))
        .def_property_readonly("fy", field(&EgoCameraIntrinsics::fy))
        .def_property_readonly("cx", field(&EgoCameraIntrinsics::cx))
        .def_property_readonly("cy", field(&EgoCameraIntrinsics::cy))
        .def_property_readonly("distortion_model", field(&EgoCameraIntrinsics::distortion_model))
        .def_property_readonly("distortion", vector_field(&EgoCameraIntrinsics::distortion));

    serialized_class<EgoExtrinsics>(m, "EgoExtrinsics", "Encoded EGO payload.")
        .def(py::init(
                 [](const std::vector<float>& rotation, const std::vector<float>& translation_mm)
                 {
                     EgoExtrinsicsT native;
                     native.rotation = rotation;
                     native.translation_mm = translation_mm;
                     return pack<EgoExtrinsics>(native);
                 }),
             py::arg("rotation") = std::vector<float>{}, py::arg("translation_mm") = std::vector<float>{})
        .def_property_readonly("rotation", vector_field(&EgoExtrinsics::rotation))
        .def_property_readonly("translation_mm", vector_field(&EgoExtrinsics::translation_mm));

    serialized_class<EgoCalibration>(m, "EgoCalibration", "Encoded EGO payload.")
        .def(py::init(
                 [](const std::string& device_uid, const Serialized<EgoCameraIntrinsics>* color_left,
                    const Serialized<EgoCameraIntrinsics>* color_right, const Serialized<EgoExtrinsics>* left_to_right,
                    const std::vector<float>& accel_intrinsics, const std::vector<float>& gyro_intrinsics,
                    const Serialized<EgoExtrinsics>* accel_to_left, const Serialized<EgoExtrinsics>* gyro_to_left,
                    const std::string& raw_alignment_yaml, const std::string& raw_imu_yaml, uint32_t capture_epoch)
                 {
                     EgoCalibrationT native;
                     native.device_uid = device_uid;
                     if (color_left != nullptr && *color_left)
                     {
                         native.color_left = std::make_shared<EgoCameraIntrinsicsT>();
                         (*color_left)->UnPackTo(native.color_left.get());
                     }
                     if (color_right != nullptr && *color_right)
                     {
                         native.color_right = std::make_shared<EgoCameraIntrinsicsT>();
                         (*color_right)->UnPackTo(native.color_right.get());
                     }
                     if (left_to_right != nullptr && *left_to_right)
                     {
                         native.left_to_right = std::make_shared<EgoExtrinsicsT>();
                         (*left_to_right)->UnPackTo(native.left_to_right.get());
                     }
                     native.accel_intrinsics = accel_intrinsics;
                     native.gyro_intrinsics = gyro_intrinsics;
                     if (accel_to_left != nullptr && *accel_to_left)
                     {
                         native.accel_to_left = std::make_shared<EgoExtrinsicsT>();
                         (*accel_to_left)->UnPackTo(native.accel_to_left.get());
                     }
                     if (gyro_to_left != nullptr && *gyro_to_left)
                     {
                         native.gyro_to_left = std::make_shared<EgoExtrinsicsT>();
                         (*gyro_to_left)->UnPackTo(native.gyro_to_left.get());
                     }
                     native.raw_alignment_yaml = raw_alignment_yaml;
                     native.raw_imu_yaml = raw_imu_yaml;
                     native.capture_epoch = capture_epoch;
                     return pack<EgoCalibration>(native);
                 }),
             py::arg("device_uid") = std::string{}, py::arg("color_left") = nullptr, py::arg("color_right") = nullptr,
             py::arg("left_to_right") = nullptr, py::arg("accel_intrinsics") = std::vector<float>{},
             py::arg("gyro_intrinsics") = std::vector<float>{}, py::arg("accel_to_left") = nullptr,
             py::arg("gyro_to_left") = nullptr, py::arg("raw_alignment_yaml") = std::string{},
             py::arg("raw_imu_yaml") = std::string{}, py::arg("capture_epoch") = 0)
        .def_property_readonly("device_uid", string_field(&EgoCalibration::device_uid))
        .def_property_readonly("color_left",
                               [](const Serialized<EgoCalibration>& self) -> py::object
                               {
                                   const auto* value = self->color_left();
                                   return value != nullptr ? py::cast(self.narrow(value)) : py::none();
                               })
        .def_property_readonly("color_right",
                               [](const Serialized<EgoCalibration>& self) -> py::object
                               {
                                   const auto* value = self->color_right();
                                   return value != nullptr ? py::cast(self.narrow(value)) : py::none();
                               })
        .def_property_readonly("left_to_right",
                               [](const Serialized<EgoCalibration>& self) -> py::object
                               {
                                   const auto* value = self->left_to_right();
                                   return value != nullptr ? py::cast(self.narrow(value)) : py::none();
                               })
        .def_property_readonly("accel_intrinsics", vector_field(&EgoCalibration::accel_intrinsics))
        .def_property_readonly("gyro_intrinsics", vector_field(&EgoCalibration::gyro_intrinsics))
        .def_property_readonly("accel_to_left",
                               [](const Serialized<EgoCalibration>& self) -> py::object
                               {
                                   const auto* value = self->accel_to_left();
                                   return value != nullptr ? py::cast(self.narrow(value)) : py::none();
                               })
        .def_property_readonly("gyro_to_left",
                               [](const Serialized<EgoCalibration>& self) -> py::object
                               {
                                   const auto* value = self->gyro_to_left();
                                   return value != nullptr ? py::cast(self.narrow(value)) : py::none();
                               })
        .def_property_readonly("raw_alignment_yaml", string_field(&EgoCalibration::raw_alignment_yaml))
        .def_property_readonly("raw_imu_yaml", string_field(&EgoCalibration::raw_imu_yaml))
        .def_property_readonly("capture_epoch", field(&EgoCalibration::capture_epoch));

    bind_record<EgoCalibrationRecord, EgoCalibration>(m, "EgoCalibrationRecord", "EgoCalibration");

    py::enum_<EgoCaptureHealth>(m, "EgoCaptureHealth")
        .value("Healthy", EgoCaptureHealth_Healthy)
        .value("Warning", EgoCaptureHealth_Warning)
        .value("Incomplete", EgoCaptureHealth_Incomplete);

    py::enum_<EgoConnectionState>(m, "EgoConnectionState")
        .value("Connected", EgoConnectionState_Connected)
        .value("Recovering", EgoConnectionState_Recovering)
        .value("Recovered", EgoConnectionState_Recovered)
        .value("Failed", EgoConnectionState_Failed);

    py::class_<EgoDevicePropertyValue>(m, "EgoDevicePropertyValue")
        .def(py::init<int32_t, double>(), py::arg("property_id") = 0, py::arg("value") = 0.0)
        .def_property("property_id", &EgoDevicePropertyValue::property_id, &EgoDevicePropertyValue::mutate_property_id)
        .def_property("value", &EgoDevicePropertyValue::value, &EgoDevicePropertyValue::mutate_value);

    serialized_class<EgoDeviceState>(m, "EgoDeviceState", "Encoded EGO payload.")
        .def(py::init(
                 [](uint64_t sequence_number, const std::string& device_uid, int32_t work_mode, uint64_t status_flags,
                    uint64_t error_flags, uint64_t storage_free_bytes, float temperature_c,
                    const std::vector<EgoDevicePropertyValue>& properties, EgoCaptureHealth capture_health,
                    const std::string& failure_reason, uint32_t queue_capacity, uint32_t queue_peak,
                    uint64_t dropped_events, uint32_t capture_epoch, EgoConnectionState connection_state,
                    uint32_t reconnect_attempt)
                 {
                     EgoDeviceStateT native;
                     native.sequence_number = sequence_number;
                     native.device_uid = device_uid;
                     native.work_mode = work_mode;
                     native.status_flags = status_flags;
                     native.error_flags = error_flags;
                     native.storage_free_bytes = storage_free_bytes;
                     native.temperature_c = temperature_c;
                     native.properties = properties;
                     native.capture_health = capture_health;
                     native.failure_reason = failure_reason;
                     native.queue_capacity = queue_capacity;
                     native.queue_peak = queue_peak;
                     native.dropped_events = dropped_events;
                     native.capture_epoch = capture_epoch;
                     native.connection_state = connection_state;
                     native.reconnect_attempt = reconnect_attempt;
                     return pack<EgoDeviceState>(native);
                 }),
             py::arg("sequence_number") = 0, py::arg("device_uid") = std::string{}, py::arg("work_mode") = 0,
             py::arg("status_flags") = 0, py::arg("error_flags") = 0, py::arg("storage_free_bytes") = 0,
             py::arg("temperature_c") = 0.0f, py::arg("properties") = std::vector<EgoDevicePropertyValue>{},
             py::arg("capture_health") = EgoCaptureHealth_Healthy, py::arg("failure_reason") = std::string{},
             py::arg("queue_capacity") = 0, py::arg("queue_peak") = 0, py::arg("dropped_events") = 0,
             py::arg("capture_epoch") = 0, py::arg("connection_state") = EgoConnectionState_Connected,
             py::arg("reconnect_attempt") = 0)
        .def_property_readonly("sequence_number", field(&EgoDeviceState::sequence_number))
        .def_property_readonly("device_uid", string_field(&EgoDeviceState::device_uid))
        .def_property_readonly("work_mode", field(&EgoDeviceState::work_mode))
        .def_property_readonly("status_flags", field(&EgoDeviceState::status_flags))
        .def_property_readonly("error_flags", field(&EgoDeviceState::error_flags))
        .def_property_readonly("storage_free_bytes", field(&EgoDeviceState::storage_free_bytes))
        .def_property_readonly("temperature_c", field(&EgoDeviceState::temperature_c))
        .def_property_readonly("properties",
                               [](const Serialized<EgoDeviceState>& self)
                               {
                                   std::vector<EgoDevicePropertyValue> values;
                                   const auto* encoded = self->properties();
                                   if (encoded != nullptr)
                                   {
                                       values.reserve(encoded->size());
                                       for (const auto* value : *encoded)
                                       {
                                           values.push_back(*value);
                                       }
                                   }
                                   return values;
                               })
        .def_property_readonly("capture_health", field(&EgoDeviceState::capture_health))
        .def_property_readonly("failure_reason", string_field(&EgoDeviceState::failure_reason))
        .def_property_readonly("queue_capacity", field(&EgoDeviceState::queue_capacity))
        .def_property_readonly("queue_peak", field(&EgoDeviceState::queue_peak))
        .def_property_readonly("dropped_events", field(&EgoDeviceState::dropped_events))
        .def_property_readonly("capture_epoch", field(&EgoDeviceState::capture_epoch))
        .def_property_readonly("connection_state", field(&EgoDeviceState::connection_state))
        .def_property_readonly("reconnect_attempt", field(&EgoDeviceState::reconnect_attempt));

    bind_record<EgoDeviceStateRecord, EgoDeviceState>(m, "EgoDeviceStateRecord", "EgoDeviceState");
}

} // namespace core
