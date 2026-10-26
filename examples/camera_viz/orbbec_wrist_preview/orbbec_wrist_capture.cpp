// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// SDK-only Orbbec Wrist Color capture. The source producer owns reads and cleanup;
// recording and IMU acquisition remain in the Orbbec Wrist plugin.

#include <libobsensor/ObSensor.hpp>
#include <pybind11/pybind11.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace py = pybind11;

namespace
{

OBFormat format_from_name(const std::string& name)
{
    if (name == "mjpg" || name == "mjpeg")
        return OB_FORMAT_MJPG;
    if (name == "h264")
        return OB_FORMAT_H264;
    if (name == "h265" || name == "hevc")
        return OB_FORMAT_H265;
    throw std::invalid_argument("format must be mjpg, h264, or h265");
}

int64_t monotonic_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// The source producer serializes reads and close; never close during a live read.
class Capture
{
public:
    Capture(std::string serial_number, uint32_t width, uint32_t height, uint32_t fps, std::string format)
        : width_(width), height_(height), format_(format_from_name(format))
    {
#if defined(__linux__) || defined(__ANDROID__)
        context_->setUvcBackendType(OB_UVC_BACKEND_TYPE_LIBUVC);
#endif
        if (width != 1600 || height != 1200 || fps == 0)
            throw std::invalid_argument("Orbbec Wrist requires 1600x1200 and positive fps");
        const auto devices = context_->queryDeviceList();
        for (uint32_t index = 0; index < devices->getCount(); ++index)
        {
            if (devices->getVid(index) != 0x2bc5 || devices->getPid(index) != 0x1203)
                continue;
            if (!serial_number.empty() && serial_number != devices->getSerialNumber(index))
                continue;
            if (device_)
                throw std::runtime_error("Multiple Orbbec Wrist cameras found; set serial_number");
            device_ = devices->getDevice(index);
            selected_serial_ = devices->getSerialNumber(index);
        }
        if (!device_)
            throw std::runtime_error("Orbbec Wrist not found: " + serial_number);
        pipeline_ = std::make_unique<ob::Pipeline>(device_);
        auto config = std::make_shared<ob::Config>();
        config->enableStream(
            device_->getSensor(OB_SENSOR_COLOR)->getStreamProfileList()->getVideoStreamProfile(width, height, format_, fps));
        pipeline_->start(config);
        if (format_ == OB_FORMAT_MJPG)
        {
            converter_ = std::make_unique<ob::FormatConvertFilter>();
            converter_->setFormatConvertType(FORMAT_MJPG_TO_RGB);
        }
    }

    ~Capture()
    {
        try
        {
            close();
        }
        catch (const ob::Error&)
        {
        }
    }

    void close()
    {
        if (pipeline_)
        {
            // A confirmed removed instance has deactivated SDK components; release it through SDK destruction.
            // Connected-device stop errors still propagate through the existing source lifecycle.
            if (!device_disconnected_)
                pipeline_->stop();
            pipeline_.reset();
        }
        converter_.reset();
        device_.reset();
        context_.reset();
    }

    const std::string& serial_number() const
    {
        return selected_serial_;
    }

    py::object next_frame(uint32_t timeout_ms)
    {
        if (!pipeline_)
            throw std::runtime_error("Orbbec Wrist capture is closed");
        if (device_disconnected_)
            throw std::runtime_error("Orbbec Wrist device " + selected_serial_ + " disconnected");
        std::shared_ptr<ob::FrameSet> frame_set;
        {
            py::gil_scoped_release release;
            frame_set = pipeline_->waitForFrameset(timeout_ms);
        }
        if (!frame_set)
            return no_frame();
        const auto timestamp_ns = monotonic_ns();
        const auto raw = frame_set->getFrame(OB_FRAME_COLOR);
        if (!raw)
            return no_frame();
        const auto frame = raw->as<ob::VideoFrame>();
        if (frame->getWidth() != width_ || frame->getHeight() != height_ || frame->getFormat() != format_)
            throw std::runtime_error("Orbbec Wrist returned a different Color profile");
        const auto payload = bytes(frame);
        if (payload.is_none())
            return py::none();
        py::dict result;
        result["data"] = payload;
        // Host retrieval time is a preview proxy, not device exposure time.
        result["timestamp_ns"] = timestamp_ns;
        result["device_timestamp_us"] = frame->getTimeStampUs();
        result["sequence"] = frame->getIndex();
        return std::move(result);
    }

private:
    py::object no_frame()
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_presence_check_)
        {
            next_presence_check_ = now + std::chrono::seconds(1);
            bool present = false;
            {
                py::gil_scoped_release release;
                const auto devices = context_->queryDeviceList();
                for (uint32_t index = 0; index < devices->getCount(); ++index)
                    if (selected_serial_ == devices->getSerialNumber(index))
                        present = true;
            }
            // None also represents transient timeout or missing Color frames; only confirmed UID loss means disconnect.
            if (!present)
            {
                device_disconnected_ = true;
                throw std::runtime_error("Orbbec Wrist device " + selected_serial_ + " disconnected");
            }
        }
        return py::none();
    }

    py::object bytes(const std::shared_ptr<ob::VideoFrame>& frame)
    {
        std::shared_ptr<ob::VideoFrame> output = frame;
        if (format_ == OB_FORMAT_MJPG)
        {
            std::shared_ptr<ob::Frame> converted;
            {
                py::gil_scoped_release release;
                converted = converter_->process(frame);
            }
            // The SDK rejects undecodable JPEGs with a null frame; drop the frame.
            if (!converted)
                return py::none();
            output = converted->as<ob::VideoFrame>();
            const auto rgb_bytes = static_cast<uint64_t>(width_) * height_ * 3;
            if (output->getFormat() != OB_FORMAT_RGB || output->getWidth() != width_ ||
                output->getHeight() != height_ || output->getDataSize() != rgb_bytes || !output->getData())
                throw std::runtime_error("Orbbec Wrist MJPEG conversion did not return the requested RGB frame");
        }
        return py::bytes(reinterpret_cast<const char*>(output->getData()), output->getDataSize());
    }

    std::unique_ptr<ob::Context> context_ = std::make_unique<ob::Context>();
    std::shared_ptr<ob::Device> device_;
    std::unique_ptr<ob::Pipeline> pipeline_;
    std::unique_ptr<ob::FormatConvertFilter> converter_;
    std::string selected_serial_;
    std::chrono::steady_clock::time_point next_presence_check_{};
    bool device_disconnected_ = false;
    uint32_t width_;
    uint32_t height_;
    OBFormat format_;
};

} // namespace

PYBIND11_MODULE(_orbbec_wrist_capture, module)
{
    module.doc() = "Native Orbbec Wrist mono capture for camera_viz";
    py::class_<Capture>(module, "Capture")
        .def(py::init<std::string, uint32_t, uint32_t, uint32_t, std::string>(), py::arg("serial_number") = "",
             py::arg("width") = 1600, py::arg("height") = 1200, py::arg("fps") = 30, py::arg("format") = "h264",
             py::call_guard<py::gil_scoped_release>())
        // SDK shutdown must not prevent the consumer thread from joining.
        .def("close", &Capture::close, py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("serial_number", &Capture::serial_number)
        .def("next_frame", &Capture::next_frame, py::arg("timeout_ms") = 100);
}
