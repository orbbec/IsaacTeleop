// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <orbbec_wrist/core/preview.hpp>

#include <SDL.h>
extern "C"
{
#include <libavcodec/avcodec.h>
}

#include <array>
#include <chrono>
#include <cstring>
#include <thread>

using namespace plugins::wrist;
namespace
{
VideoFrame red_jpeg()
{
    const auto* codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
    REQUIRE(codec);
    auto* context = avcodec_alloc_context3(codec);
    auto* image = av_frame_alloc();
    auto* packet = av_packet_alloc();
    struct Cleanup
    {
        AVCodecContext*& context;
        AVFrame*& image;
        AVPacket*& packet;
        ~Cleanup()
        {
            av_packet_free(&packet);
            av_frame_free(&image);
            avcodec_free_context(&context);
        }
    } cleanup{ context, image, packet };
    REQUIRE(context);
    REQUIRE(image);
    REQUIRE(packet);
    context->width = context->height = 16;
    context->time_base = { 1, 30 };
    context->pix_fmt = AV_PIX_FMT_YUVJ420P;
    REQUIRE(avcodec_open2(context, codec, nullptr) == 0);
    image->format = context->pix_fmt;
    image->width = image->height = 16;
    REQUIRE(av_frame_get_buffer(image, 0) == 0);
    for (int plane = 0; plane < 3; ++plane)
        for (int row = 0; row < (plane ? 8 : 16); ++row)
            std::memset(image->data[plane] + row * image->linesize[plane],
                        plane == 0 ? 76 :
                        plane == 1 ? 85 :
                                     255,
                        plane ? 8 : 16);
    REQUIRE(avcodec_send_frame(context, image) == 0);
    REQUIRE(avcodec_receive_packet(context, packet) == 0);
    VideoFrame result;
    result.profile = { PixelFormat::Mjpg, 16, 16, 30 };
    result.capture_epoch = 1;
    result.encoded_data.assign(packet->data, packet->data + packet->size);
    return result;
}

SDL_Window* created_window()
{
    SDL_Event event;
    while (SDL_PollEvent(&event))
        if (event.type == SDL_WINDOWEVENT)
            if (auto* window = SDL_GetWindowFromID(event.window.windowID))
                return window;
    return nullptr;
}
} // namespace

TEST_CASE("wrist preview decodes and presents MJPEG without camera hardware", "[wrist][preview][unit]")
{
    REQUIRE(SDL_setenv("SDL_VIDEODRIVER", "dummy", 1) == 0);
    Preview preview;
    auto* window = created_window();
    REQUIRE(window);
    REQUIRE(std::strcmp(SDL_GetWindowTitle(window), "Orbbec Wrist Color Preview - Esc/Q to stop recording") == 0);
    auto* renderer = SDL_GetRenderer(window);
    REQUIRE(renderer);
    auto frame = red_jpeg();
    preview.submit(frame);
    // Submit owns a copy; acquisition buffers may be reused immediately.
    frame.encoded_data.assign(frame.encoded_data.size(), 0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    std::array<uint8_t, 3> pixel{};
    do
    {
        preview.update();
        const SDL_Rect center{ 400, 300, 1, 1 };
        REQUIRE(SDL_RenderReadPixels(renderer, &center, SDL_PIXELFORMAT_RGB24, pixel.data(), 3) == 0);
        if (pixel[0] > 200 && pixel[1] < 40 && pixel[2] < 40)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    REQUIRE(pixel[0] > 200);
    REQUIRE(pixel[1] < 40);
    REQUIRE(pixel[2] < 40);
    REQUIRE_FALSE(preview.closed());
}

TEST_CASE("wrist preview operator exit requests normal recording shutdown", "[wrist][preview][unit]")
{
    const auto type = GENERATE(SDL_QUIT, SDL_KEYDOWN, SDL_WINDOWEVENT);
    REQUIRE(SDL_setenv("SDL_VIDEODRIVER", "dummy", 1) == 0);
    Preview preview;
    REQUIRE(created_window());
    SDL_Event event{};
    event.type = type;
    if (type == SDL_KEYDOWN)
        event.key.keysym.sym = SDLK_ESCAPE;
    if (type == SDL_WINDOWEVENT)
        event.window.event = SDL_WINDOWEVENT_CLOSE;
    REQUIRE(SDL_PushEvent(&event) == 1);
    // SDL can finish the current poll cycle before returning an injected event.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!preview.closed() && std::chrono::steady_clock::now() < deadline)
    {
        preview.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    REQUIRE(preview.closed());
}

TEST_CASE("wrist preview display failure leaves recording running", "[wrist][preview][unit]")
{
    REQUIRE(SDL_setenv("SDL_VIDEODRIVER", "isaaccapture_missing_video_driver", 1) == 0);
    {
        Preview preview;
        REQUIRE_FALSE(preview.closed());
        preview.submit(red_jpeg());
        preview.update();
        REQUIRE_FALSE(preview.closed());
    }
    REQUIRE(SDL_setenv("SDL_VIDEODRIVER", "dummy", 1) == 0);
}
