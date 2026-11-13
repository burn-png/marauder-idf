#include "include/media_player.hpp"
#include "include/display.hpp"
#include "include/storage.hpp"
#include "jpeg_roi_decoder.h"
#include "png.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <setjmp.h>

namespace {
constexpr char kTag[] = "media";
constexpr int kImageTop = 34;
constexpr int kImageHeight = 260;
constexpr size_t kMaxPngDimension = 2048;
// The JPEG work and chunk buffers are allocated per image (and freed after) instead of sitting in static DRAM: on this
// board static data competes with Wi-Fi/BT for the same ~180 KB of internal RAM.
constexpr unsigned kMaxViewWidth = 480;           // widest decoded (post 1/8-scale) image; chunk buffer = width*16*2 B

struct PngContext {
    marauder::StorageReader reader{};
    png_structp png{};
    png_infop info{};
    uint8_t *row{};
    char error[48]{};
};

struct JpegContext {
    marauder::StorageReader reader{};
    size_t bytes_since_yield{};
    unsigned source_width{};
    unsigned source_height{};
    unsigned output_width{};
    unsigned output_height{};
    unsigned next_output_y{};
    int x_offset{};
    int y_offset{};
};

struct DecodeRequest {
    TaskHandle_t caller{};
    char name[80]{};
    char message[48]{};
    bool shown{};
};

void set_message(char *out, size_t cap, const char *text) {
    if (out && cap) snprintf(out, cap, "%s", text ? text : "MEDIA ERROR");
}

bool ends_with(const char *text, const char *extension) {
    const size_t text_len = strlen(text), ext_len = strlen(extension);
    if (text_len < ext_len) return false;
    for (size_t i = 0; i < ext_len; ++i) {
        const unsigned char a = static_cast<unsigned char>(text[text_len - ext_len + i]);
        const unsigned char b = static_cast<unsigned char>(extension[i]);
        if (std::tolower(a) != std::tolower(b)) return false;
    }
    return true;
}

size_t jpeg_read(uint8_t *dst, size_t max, void *ctx) {
    auto *source = static_cast<JpegContext *>(ctx);
    const size_t count = !dst
        ? (marauder::storage_reader_skip(&source->reader, max) ? max : 0)
        : marauder::storage_reader_read(&source->reader, dst, max);
    source->bytes_since_yield += count;
    if (source->bytes_since_yield >= 4096) {
        source->bytes_since_yield %= 4096;
        vTaskDelay(1);
    }
    return count;
}

bool jpeg_chunk(const jpeg_chunk_event_t *event) {
    auto *source = event ? static_cast<JpegContext *>(event->user_data) : nullptr;
    if (!event || !event->pixels || !event->width || !event->byte_count || !source ||
        event->byte_count % (static_cast<size_t>(event->width) * sizeof(uint16_t)) ||
        event->width > kMaxViewWidth ||
        source->x_offset + source->output_width > marauder::screen_width ||
        source->y_offset + source->output_height > kImageHeight) return false;
    size_t height = event->byte_count / (static_cast<size_t>(event->width) * sizeof(uint16_t));
    if (event->y >= source->source_height) return true;                         // padding rows past the image edge
    if (event->y + height > source->source_height) height = source->source_height - event->y;

    uint16_t row[marauder::screen_width];
    const auto *pixels = static_cast<const uint16_t *>(event->pixels);
    for (size_t source_row = 0; source_row < height; ++source_row) {
        const unsigned source_y = event->y + static_cast<unsigned>(source_row);
        while (source->next_output_y < source->output_height &&
               (static_cast<uint64_t>(source->next_output_y) * source->source_height) /
                       source->output_height == source_y) {
            for (unsigned x = 0; x < source->output_width; ++x) {
                unsigned source_x = static_cast<unsigned>(
                    (static_cast<uint64_t>(x) * source->source_width) / source->output_width);
                if (source_x < event->x) source_x = event->x;                     // clamp: never reject a whole image
                if (source_x >= event->x + event->width) source_x = event->x + event->width - 1;
                row[x] = pixels[source_row * event->width + source_x - event->x];
            }
            marauder::display_rgb565_image(source->x_offset,
                kImageTop + source->y_offset + static_cast<int>(source->next_output_y),
                static_cast<int>(source->output_width), 1, row);
            ++source->next_output_y;
        }
    }
    return true;
}

// Baseline/extended sequential only: the decoder cannot do progressive (SOF2) files, which many phones and web
// exports produce. Walk the marker segments to tell the user why instead of a vague "header invalid".
bool jpeg_is_progressive(const char *name) {
    marauder::StorageReader r{};
    if (!marauder::storage_reader_open(&r, name)) return false;
    bool progressive = false;
    uint8_t b[4];
    if (marauder::storage_reader_read(&r, b, 2) == 2 && b[0] == 0xFF && b[1] == 0xD8) {
        for (int guard = 0; guard < 64; ++guard) {
            if (marauder::storage_reader_read(&r, b, 1) != 1) break;
            if (b[0] != 0xFF) continue;
            uint8_t m;
            do { if (marauder::storage_reader_read(&r, &m, 1) != 1) { m = 0; break; } } while (m == 0xFF);
            if (m == 0 || m == 0x01 || (m >= 0xD0 && m <= 0xD8)) continue;
            if (m == 0xDA || m == 0xD9) break;
            if (marauder::storage_reader_read(&r, b, 2) != 2) break;
            const unsigned len = (static_cast<unsigned>(b[0]) << 8) | b[1];
            if (m == 0xC2 || m == 0xC6 || m == 0xCA || m == 0xCE) { progressive = (m == 0xC2); break; }
            if (len < 2 || !marauder::storage_reader_skip(&r, len - 2)) break;
        }
    }
    marauder::storage_reader_close(&r);
    return progressive;
}

void png_error_callback(png_structp png, png_const_charp message) {
    auto *ctx = static_cast<PngContext *>(png_get_error_ptr(png));
    if (ctx) snprintf(ctx->error, sizeof(ctx->error), "PNG: %.26s", message ? message : "decode error");
    png_longjmp(png, 1);
}

void png_warning_callback(png_structp png, png_const_charp message) {
    ESP_LOGW(kTag, "PNG warning: %s", message ? message : "unknown");
}

void png_read_callback(png_structp png, png_bytep output, png_size_t bytes) {
    auto *ctx = static_cast<PngContext *>(png_get_io_ptr(png));
    if (!ctx || marauder::storage_reader_read(&ctx->reader, output, bytes) != bytes)
        png_error(png, "truncated input");
}

bool display_png(const char *name, char *message, size_t message_size) {
    auto *ctx = new (std::nothrow) PngContext{};
    if (!ctx) {
        set_message(message, message_size, "NO MEMORY");
        return false;
    }
    if (!marauder::storage_reader_open(&ctx->reader, name)) {
        delete ctx;
        set_message(message, message_size, "SD OPEN FAILED");
        return false;
    }
    png_uint_32 width = 0, height = 0;
    int bit_depth = 0, color_type = 0, interlace = 0, compression = 0, filter = 0;
    png_size_t row_bytes = 0;
    ctx->png = png_create_read_struct(PNG_LIBPNG_VER_STRING, ctx, png_error_callback, png_warning_callback);
    if (ctx->png) ctx->info = png_create_info_struct(ctx->png);
    if (!ctx->png || !ctx->info) {
        set_message(message, message_size, "PNG INIT FAILED");
        goto cleanup;
    }
    if (setjmp(png_jmpbuf(ctx->png))) {
        set_message(message, message_size, ctx->error[0] ? ctx->error : "PNG DECODE FAILED");
        goto cleanup;
    }

    png_set_read_fn(ctx->png, ctx, png_read_callback);
    png_set_user_limits(ctx->png, kMaxPngDimension, kMaxPngDimension);
    png_set_chunk_malloc_max(ctx->png, 32768);
    png_set_chunk_cache_max(ctx->png, 64);
    png_read_info(ctx->png, ctx->info);
    if (!png_get_IHDR(ctx->png, ctx->info, &width, &height, &bit_depth, &color_type,
                      &interlace, &compression, &filter) ||
        !width || !height || width > kMaxPngDimension || height > kMaxPngDimension) {
        set_message(message, message_size, "PNG SIZE UNSUPPORTED");
        goto cleanup;
    }
    if (interlace != PNG_INTERLACE_NONE) {
        set_message(message, message_size, "INTERLACED PNG UNSUPPORTED");
        goto cleanup;
    }
    ESP_LOGI(kTag, "PNG %s: %lux%lu depth=%d type=%d internal heap=%lu decoder stack=%u",
             name, static_cast<unsigned long>(width), static_cast<unsigned long>(height),
             bit_depth, color_type,
             static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));

    if (bit_depth == 16) png_set_strip_16(ctx->png);
    if (color_type == PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(ctx->png);
    if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) png_set_expand_gray_1_2_4_to_8(ctx->png);
    if (png_get_valid(ctx->png, ctx->info, PNG_INFO_tRNS)) png_set_tRNS_to_alpha(ctx->png);
    if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
        png_set_gray_to_rgb(ctx->png);
    if (!(color_type & PNG_COLOR_MASK_ALPHA) && !png_get_valid(ctx->png, ctx->info, PNG_INFO_tRNS))
        png_set_add_alpha(ctx->png, 0xff, PNG_FILLER_AFTER);
    png_read_update_info(ctx->png, ctx->info);
    row_bytes = png_get_rowbytes(ctx->png, ctx->info);
    if (row_bytes != static_cast<png_size_t>(width) * 4) {
        set_message(message, message_size, "PNG FORMAT UNSUPPORTED");
        goto cleanup;
    }
    ctx->row = static_cast<uint8_t *>(malloc(row_bytes));
    if (!ctx->row) {
        ESP_LOGW(kTag, "PNG row allocation failed: %lu bytes, largest block %lu",
                 static_cast<unsigned long>(row_bytes),
                 static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        set_message(message, message_size, "NO MEMORY");
        goto cleanup;
    }

    {
        const double scale = std::min(static_cast<double>(marauder::screen_width) / width,
                                      static_cast<double>(kImageHeight) / height);
        const unsigned out_width = std::max(1u, static_cast<unsigned>(width * scale));
        const unsigned out_height = std::max(1u, static_cast<unsigned>(height * scale));
        const unsigned x_offset = (marauder::screen_width - out_width) / 2;
        const unsigned y_offset = (kImageHeight - out_height) / 2;
        std::array<uint16_t, marauder::screen_width> pixels{};
        unsigned next_out_y = 0;
        for (png_uint_32 y = 0; y < height; ++y) {
            png_read_row(ctx->png, ctx->row, nullptr);
            while (next_out_y < out_height &&
                   static_cast<png_uint_32>((static_cast<uint64_t>(next_out_y) * height) / out_height) == y) {
                for (unsigned x = 0; x < out_width; ++x) {
                    const unsigned src_x = static_cast<unsigned>((static_cast<uint64_t>(x) * width) / out_width);
                    const uint8_t *rgba = ctx->row + src_x * 4;
                    const unsigned alpha = rgba[3];
                    const unsigned red = (rgba[0] * alpha + 127) / 255;
                    const unsigned green = (rgba[1] * alpha + 127) / 255;
                    const unsigned blue = (rgba[2] * alpha + 127) / 255;
                    pixels[x] = static_cast<uint16_t>(((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3));
                }
                marauder::display_rgb565_image(static_cast<int>(x_offset),
                    static_cast<int>(kImageTop + y_offset + next_out_y), static_cast<int>(out_width), 1,
                    pixels.data());
                ++next_out_y;
            }
            if ((y & 7u) == 7u) vTaskDelay(1);
        }
        png_read_end(ctx->png, ctx->info);
    }
    set_message(message, message_size, "IMAGE DISPLAYED");

cleanup:
    free(ctx->row);
    if (ctx->png) png_destroy_read_struct(&ctx->png, ctx->info ? &ctx->info : nullptr, nullptr);
    marauder::storage_reader_close(&ctx->reader);
    delete ctx;
    return message && message_size && strcmp(message, "IMAGE DISPLAYED") == 0;
}

bool display_jpeg(const char *name, char *message, size_t message_size) {
    JpegContext source{};
    uint8_t *work = static_cast<uint8_t *>(malloc(JPEG_DECODER_WORK_BUF_DEFAULT));
    if (!work) {
        snprintf(message, message_size, "NO MEMORY (WORK %luB FREE)",
                 static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        return false;
    }
    bool ok = false;
    uint16_t *chunks = nullptr;
    if (!marauder::storage_reader_open(&source.reader, name)) {
        set_message(message, message_size, "SD OPEN FAILED");
        free(work);
        return false;
    }
    jpeg_image_info_t image{};
    const jpeg_decode_result_t probe = jpeg_decoder_probe(
        {jpeg_read, &source}, &image, work, JPEG_DECODER_WORK_BUF_DEFAULT);
    marauder::storage_reader_close(&source.reader);
    if (probe != JPEG_DECODE_OK || !image.width || !image.height) {
        ESP_LOGW(kTag, "JPEG probe failed for %s: %s", name, jpeg_decoder_err_to_str(probe));
        set_message(message, message_size, jpeg_is_progressive(name) ? "PROGRESSIVE JPEG UNSUPPORTED" : "JPEG HEADER INVALID");
        free(work);
        return false;
    }
    if (jpeg_is_progressive(name)) {
        set_message(message, message_size, "PROGRESSIVE JPEG UNSUPPORTED");
        free(work);
        return false;
    }
    ESP_LOGI(kTag, "JPEG %s: %ux%u internal heap=%lu largest=%lu", name, image.width, image.height,
             static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));

    // Decode at the largest reduction that still has at least a screenful of pixels (so a 12 MP photo becomes 500x375
    // at 1/8), then fit it to the screen by sampling in the chunk callback.
    unsigned scale = 0;
    while (scale < 3 && ((image.width >> (scale + 1)) >= marauder::screen_width ||
                         (image.height >> (scale + 1)) >= static_cast<unsigned>(kImageHeight))) ++scale;
    const unsigned divisor = 1u << scale;
    const unsigned scaled_width = (image.width + divisor - 1) / divisor;
    const unsigned scaled_height = (image.height + divisor - 1) / divisor;
    if (scaled_width > kMaxViewWidth) {
        ESP_LOGW(kTag, "JPEG too wide: %ux%u (limit %u px after 1/8)", image.width, image.height, kMaxViewWidth * 8);
        snprintf(message, message_size, "JPEG OVER %u PX WIDE", kMaxViewWidth * 8);
        free(work);
        return false;
    }
    const size_t chunk_bytes = JPEG_CHUNK_BUF_BYTES(scaled_width);
    chunks = static_cast<uint16_t *>(malloc(chunk_bytes));
    if (!chunks) {
        snprintf(message, message_size, "NO MEMORY: NEED %uK HAVE %luK", static_cast<unsigned>((chunk_bytes + 1023) / 1024),
                 static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024));
        free(work);
        return false;
    }
    const double display_scale = std::min(static_cast<double>(marauder::screen_width) / scaled_width,
                                          static_cast<double>(kImageHeight) / scaled_height);
    source.source_width = scaled_width;
    source.source_height = scaled_height;
    source.output_width = std::min<unsigned>(marauder::screen_width,
        std::max(1u, static_cast<unsigned>(scaled_width * display_scale)));
    source.output_height = std::min<unsigned>(kImageHeight,
        std::max(1u, static_cast<unsigned>(scaled_height * display_scale)));
    source.x_offset = (marauder::screen_width - static_cast<int>(source.output_width)) / 2;
    source.y_offset = (kImageHeight - static_cast<int>(source.output_height)) / 2;
    if (!marauder::storage_reader_open(&source.reader, name)) {
        set_message(message, message_size, "SD REOPEN FAILED");
        goto done;
    }
    ESP_LOGI(kTag, "JPEG render %ux%u -> %ux%u at 1/%u decoder scale", source.source_width, source.source_height,
             source.output_width, source.output_height, divisor);
    {
        jpeg_view_intent_t view = jpeg_view_default(static_cast<uint16_t>(scaled_width), static_cast<uint16_t>(scaled_height));
        view.reader = {jpeg_read, &source};
        view.chunk_buffer = chunks;
        view.out_format = JPEG_OUTPUT_RGB565;
        view.scale = static_cast<jpeg_decode_scale_t>(scale);
        const jpeg_decode_result_t result = jpeg_decoder_decode_view(
            &view, work, JPEG_DECODER_WORK_BUF_DEFAULT, jpeg_chunk, nullptr, &source);
        marauder::storage_reader_close(&source.reader);
        if (result != JPEG_DECODE_OK) {
            ESP_LOGW(kTag, "JPEG decode failed for %s (%ux%u, 1/%u): %s", name, image.width, image.height, divisor,
                     jpeg_decoder_err_to_str(result));
            set_message(message, message_size, jpeg_decoder_err_to_str(result));
            goto done;
        }
    }
    set_message(message, message_size, "IMAGE DISPLAYED");
    ok = true;
done:
    free(chunks);
    free(work);
    return ok;
}

void media_decode_task(void *argument) {
    auto *request = static_cast<DecodeRequest *>(argument);
    marauder::display_fill_rect(0, kImageTop, marauder::screen_width, kImageHeight, 0x0000);
    switch (marauder::media_file_kind(request->name)) {
        case marauder::MediaKind::JPEG:
            request->shown = display_jpeg(request->name, request->message, sizeof(request->message));
            break;
        case marauder::MediaKind::PNG:
            request->shown = display_png(request->name, request->message, sizeof(request->message));
            break;
        default:
            set_message(request->message, sizeof(request->message), "IMAGE FORMAT UNSUPPORTED");
            break;
    }
    ESP_LOGI(kTag, "image %s: %s, internal heap %lu, decoder stack watermark %u",
             request->shown ? "rendered" : "failed", request->message,
             static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
    xTaskNotifyGive(request->caller);
    vTaskDelete(nullptr);
}
}

namespace marauder {
size_t media_list_files(char names[][64], size_t capacity) {
    if (!names || !capacity) return 0;
    const size_t count = storage_list_media_files(names, capacity);
    for (size_t i = 1; i < count; ++i) {
        char current[64];
        memcpy(current, names[i], sizeof(current));
        size_t j = i;
        while (j && strcasecmp(names[j - 1], current) > 0) {
            memcpy(names[j], names[j - 1], sizeof(current));
            --j;
        }
        memcpy(names[j], current, sizeof(current));
    }
    return count;
}

MediaKind media_file_kind(const char *name) {
    if (!name) return MediaKind::OTHER;
    if (ends_with(name, ".jpg") || ends_with(name, ".jpeg")) return MediaKind::JPEG;
    if (ends_with(name, ".png")) return MediaKind::PNG;
    if (ends_with(name, ".mp4")) return MediaKind::MP4;
    return MediaKind::OTHER;
}

bool media_display_image(const char *name, char *message, size_t message_size) {
    if (!name || !message || !message_size) return false;
    if (strnlen(name, 80) >= 80) {
        set_message(message, message_size, "FILENAME TOO LONG");
        return false;
    }
    DecodeRequest request{};
    request.caller = xTaskGetCurrentTaskHandle();
    snprintf(request.name, sizeof(request.name), "%s", name);
    const uint32_t stack = media_file_kind(name) == MediaKind::PNG ? 16384 : 12288;
    if (xTaskCreate(media_decode_task, "media_decode", stack, &request, 4, nullptr) != pdPASS) {
        snprintf(message, message_size, "NO MEMORY: NEED %luK HAVE %luK", static_cast<unsigned long>(stack / 1024),
                 static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024));
        return false;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    set_message(message, message_size, request.message);
    return request.shown;
}
}
