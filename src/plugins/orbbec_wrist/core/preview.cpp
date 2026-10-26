// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "preview.hpp"

#include <log_bridge/logger.hpp>

#include <SDL.h>
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

namespace plugins::wrist
{
namespace
{
struct Image
{
    int width = 0;
    int height = 0;
    std::vector<uint8_t> pixels;
};

class Decoder
{
public:
    explicit Decoder(PixelFormat format)
    {
        const auto id = format == PixelFormat::Mjpg ? AV_CODEC_ID_MJPEG :
                        format == PixelFormat::H264 ? AV_CODEC_ID_H264 :
                                                      AV_CODEC_ID_HEVC;
        const auto* codec = avcodec_find_decoder(id);
        if (!codec)
            throw std::runtime_error("FFmpeg preview decoder unavailable");
        context_ = avcodec_alloc_context3(codec);
        if (!context_)
            throw std::runtime_error("Cannot allocate preview decoder");
        context_->thread_count = 1;
        if (avcodec_open2(context_, codec, nullptr) < 0)
        {
            avcodec_free_context(&context_);
            throw std::runtime_error("Cannot open preview decoder");
        }
        frame_ = av_frame_alloc();
        packet_ = av_packet_alloc();
        if (!frame_ || !packet_)
        {
            av_frame_free(&frame_);
            av_packet_free(&packet_);
            avcodec_free_context(&context_);
            throw std::runtime_error("Cannot allocate preview frame or packet");
        }
    }

    ~Decoder()
    {
        sws_freeContext(converter_);
        av_packet_free(&packet_);
        av_frame_free(&frame_);
        avcodec_free_context(&context_);
    }

    bool decode(const VideoFrame& input, Image& image)
    {
        if (input.encoded_data.size() > static_cast<size_t>(std::numeric_limits<int>::max()))
            throw std::runtime_error("Preview packet exceeds FFmpeg size limit");
        av_packet_unref(packet_);
        // Owned packets include the zero padding required by FFmpeg's bitstream readers.
        if (av_new_packet(packet_, static_cast<int>(input.encoded_data.size())) < 0)
            throw std::runtime_error("Cannot allocate preview packet data");
        std::memcpy(packet_->data, input.encoded_data.data(), input.encoded_data.size());
        bool produced = false;
        int result = avcodec_send_packet(context_, packet_);
        if (result == AVERROR(EAGAIN))
        {
            produced = receive(input, image);
            result = avcodec_send_packet(context_, packet_);
        }
        if (result < 0)
            throw std::runtime_error("FFmpeg rejected preview video packet");
        return receive(input, image) || produced;
    }

private:
    bool receive(const VideoFrame& input, Image& image)
    {
        bool produced = false;
        while (true)
        {
            const int result = avcodec_receive_frame(context_, frame_);
            if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
                return produced;
            if (result < 0 || frame_->decode_error_flags || frame_->width <= 0 || frame_->height <= 0 ||
                static_cast<uint32_t>(frame_->width) != input.profile.width ||
                static_cast<uint32_t>(frame_->height) != input.profile.height)
                throw std::runtime_error("Cannot decode preview image with selected dimensions");
            const double scale = std::min({ 1.0, 800.0 / frame_->width, 600.0 / frame_->height });
            image.width = std::max(1, static_cast<int>(frame_->width * scale));
            image.height = std::max(1, static_cast<int>(frame_->height * scale));
            image.pixels.resize(static_cast<size_t>(image.width) * image.height * 3);
            converter_ = sws_getCachedContext(converter_, frame_->width, frame_->height,
                                              static_cast<AVPixelFormat>(frame_->format), image.width, image.height,
                                              AV_PIX_FMT_RGB24, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
            if (!converter_)
                throw std::runtime_error("Cannot allocate preview RGB converter");
            uint8_t* destinations[] = { image.pixels.data() };
            int strides[] = { image.width * 3 };
            if (sws_scale(converter_, frame_->data, frame_->linesize, 0, frame_->height, destinations, strides) !=
                image.height)
                throw std::runtime_error("Cannot convert preview image to RGB");
            produced = true;
        }
    }

    AVCodecContext* context_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVPacket* packet_ = nullptr;
    SwsContext* converter_ = nullptr;
};
} // namespace

class Preview::Impl
{
public:
    Impl()
    {
        try
        {
            if (SDL_InitSubSystem(SDL_INIT_VIDEO) != 0)
                throw std::runtime_error(SDL_GetError());
            initialized_ = true;
            window_ = SDL_CreateWindow("Orbbec Wrist Color Preview - Esc/Q to stop recording", SDL_WINDOWPOS_CENTERED,
                                       SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_RESIZABLE);
            if (window_)
            {
                renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
                if (!renderer_)
                    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
            }
            if (!window_ || !renderer_)
                throw std::runtime_error(SDL_GetError());
            worker_ = std::thread([this] { run(); });
        }
        catch (const std::exception& error)
        {
            disable(error.what());
            destroy_window();
        }
    }

    ~Impl()
    {
        stop_.store(true);
        wake_.notify_all();
        if (worker_.joinable())
            worker_.join();
        destroy_window();
    }

    void submit(const VideoFrame& frame) noexcept
    {
        if (stop_.load())
            return;
        try
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (stop_.load())
                return;
            if (frame.profile.format == PixelFormat::Mjpg)
            {
                queue_.clear();
                queued_bytes_ = 0;
            }
            // Prediction packets stay ordered; overload disables preview rather than delaying capture.
            if (queue_.size() >= 64 || frame.encoded_data.size() > kMaxBytes ||
                queued_bytes_ > kMaxBytes - frame.encoded_data.size())
                throw std::runtime_error("preview queue exceeded 64 packets or 64 MiB");
            queue_.push_back(frame);
            queued_bytes_ += frame.encoded_data.size();
            wake_.notify_one();
        }
        catch (const std::exception& error)
        {
            disable(error.what());
        }
    }

    void update() noexcept
    {
        if (stop_.load())
        {
            destroy_window();
            return;
        }
        try
        {
            SDL_Event event;
            bool redraw = false;
            while (SDL_PollEvent(&event))
            {
                redraw = redraw || event.type == SDL_WINDOWEVENT;
                if (event.type == SDL_QUIT ||
                    (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) ||
                    (event.type == SDL_KEYDOWN && (event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q)))
                {
                    closed_ = true;
                    stop_.store(true);
                    wake_.notify_all();
                    destroy_window();
                    return;
                }
            }
            std::optional<Image> image;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                image.swap(latest_);
            }
            if (!image && !redraw)
                return;
            if (image)
            {
                if (!texture_ || width_ != image->width || height_ != image->height)
                {
                    if (texture_)
                        SDL_DestroyTexture(texture_);
                    texture_ = SDL_CreateTexture(
                        renderer_, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, image->width, image->height);
                    width_ = image->width;
                    height_ = image->height;
                    if (!texture_ || SDL_RenderSetLogicalSize(renderer_, width_, height_) != 0)
                        throw std::runtime_error(SDL_GetError());
                }
                if (SDL_UpdateTexture(texture_, nullptr, image->pixels.data(), width_ * 3) != 0)
                    throw std::runtime_error(SDL_GetError());
            }
            if (SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255) != 0 || SDL_RenderClear(renderer_) != 0 ||
                (texture_ && SDL_RenderCopy(renderer_, texture_, nullptr, nullptr) != 0))
                throw std::runtime_error(SDL_GetError());
            SDL_RenderPresent(renderer_);
        }
        catch (const std::exception& error)
        {
            disable(error.what());
            destroy_window();
        }
    }

    bool closed() const
    {
        return closed_;
    }

private:
    void disable(const char* reason) noexcept
    {
        if (stop_.exchange(true))
            return;
        wake_.notify_all();
        try
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.clear();
            queued_bytes_ = 0;
            latest_.reset();
            isaaccapture::Logger::get("isaaccapture.plugins.orbbec_wrist.Preview")
                ->warn("Orbbec Wrist preview disabled: {}; recording continues", reason);
        }
        catch (...)
        {
        }
    }

    void run() noexcept
    {
        try
        {
            std::unique_ptr<Decoder> decoder;
            std::optional<VideoFrame> previous;
            while (!stop_.load())
            {
                VideoFrame frame;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    wake_.wait(lock, [this] { return stop_.load() || !queue_.empty(); });
                    if (stop_.load())
                        return;
                    frame = std::move(queue_.front());
                    queue_.pop_front();
                    queued_bytes_ -= frame.encoded_data.size();
                }
                if (!previous || previous->capture_epoch != frame.capture_epoch ||
                    previous->sequence + 1 != frame.sequence || previous->profile != frame.profile)
                    decoder = std::make_unique<Decoder>(frame.profile.format);
                Image image;
                if (decoder->decode(frame, image))
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_ = std::move(image);
                }
                frame.encoded_data.clear();
                previous = std::move(frame);
            }
        }
        catch (const std::exception& error)
        {
            disable(error.what());
        }
    }

    void destroy_window() noexcept
    {
        if (texture_)
            SDL_DestroyTexture(std::exchange(texture_, nullptr));
        if (renderer_)
            SDL_DestroyRenderer(std::exchange(renderer_, nullptr));
        if (window_)
            SDL_DestroyWindow(std::exchange(window_, nullptr));
        if (initialized_)
        {
            SDL_QuitSubSystem(SDL_INIT_VIDEO);
            initialized_ = false;
        }
    }

    static constexpr size_t kMaxBytes = 64 * 1024 * 1024;
    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    SDL_Texture* texture_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    bool initialized_ = false;
    bool closed_ = false;
    std::atomic<bool> stop_{ false };
    std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<VideoFrame> queue_;
    size_t queued_bytes_ = 0;
    std::optional<Image> latest_;
    std::thread worker_;
};

Preview::Preview() : impl_(std::make_unique<Impl>())
{
}
Preview::~Preview() = default;
void Preview::submit(const VideoFrame& frame) noexcept
{
    impl_->submit(frame);
}
void Preview::update() noexcept
{
    impl_->update();
}
bool Preview::closed() const
{
    return impl_->closed();
}
} // namespace plugins::wrist
