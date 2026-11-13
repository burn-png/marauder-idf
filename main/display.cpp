#include "display.hpp"
#include "font_cp437.hpp"
#include "pins.hpp"
#include "spi_gate.hpp"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_check.h"
#include <algorithm>
#include <cstring>

namespace {
spi_device_handle_t s_tft;
alignas(4) DMA_ATTR uint8_t s_line[240 * 2];
alignas(4) DMA_ATTR uint8_t s_fill[256 * 2];
alignas(4) DMA_ATTR uint8_t s_text_line[240 * 2];
constexpr uint16_t kBlack = 0x0000;
// Keep the title/divider and spectrum trace out of the scrollable waterfall.
constexpr int kWaterTop = 76;
constexpr int kWaterHeight = 220;
int s_scroll = 0;
struct TextCache { int x, y; uint16_t fg, bg; char text[31]; bool valid; };
TextCache s_text_cache[32]{};
size_t s_text_cache_next{};

void invalidate_text(int x, int y, int width, int height) {
    for (auto &e : s_text_cache) {
        if (!e.valid) continue;
        const int right=e.x+(int)strlen(e.text)*8;
        if (x < right && x+width > e.x && y < e.y+8 && y+height > e.y) e.valid=false;
    }
}

void tx(bool data, const uint8_t *bytes, size_t size) {
    gpio_set_level(marauder::pins::tft_dc, data);
    spi_transaction_t t{};
    t.length = size * 8;
    t.tx_buffer = bytes;
    spi_device_transmit(s_tft, &t);
}
void cmd(uint8_t c, const uint8_t *p = nullptr, size_t n = 0) {
    tx(false, &c, 1);
    if (p && n) tx(true, p, n);
}
void set_window(int x0, int y0, int x1, int y1) {
    uint8_t p[4];
    const uint8_t c = 0x2a;
    p[0] = x0 >> 8; p[1] = x0; p[2] = x1 >> 8; p[3] = x1;
    tx(false, &c, 1); tx(true, p, sizeof(p));
    const uint8_t r = 0x2b;
    p[0] = y0 >> 8; p[1] = y0; p[2] = y1 >> 8; p[3] = y1;
    tx(false, &r, 1); tx(true, p, sizeof(p));
    const uint8_t m = 0x2c;
    tx(false, &m, 1);
}
void fill_rect(int x0, int y0, int x1, int y1, uint16_t color) {
    invalidate_text(x0,y0,x1-x0+1,y1-y0+1);
    set_window(x0, y0, x1, y1);
    size_t pixels = (size_t)(x1 - x0 + 1) * (size_t)(y1 - y0 + 1);
    for (size_t i = 0; i < 256; ++i) {
        s_fill[i * 2] = (uint8_t)(color >> 8);
        s_fill[i * 2 + 1] = (uint8_t)color;
    }
    gpio_set_level(marauder::pins::tft_dc, 1);
    while (pixels) {
        size_t n = pixels > 128 ? 128 : pixels;
        spi_transaction_t t{};
        t.length = n * 16;
        t.tx_buffer = s_fill;
        spi_device_transmit(s_tft, &t);
        pixels -= n;
    }
}
void push_scanline(int y, const uint16_t *colors) {
    set_window(0, y, 239, y);
    for (int x = 0; x < 240; ++x) {
        s_line[x * 2] = (uint8_t)(colors[x] >> 8);
        s_line[x * 2 + 1] = (uint8_t)colors[x];
    }
    gpio_set_level(marauder::pins::tft_dc, 1);
    spi_transaction_t t{};
    t.length = sizeof(s_line) * 8;
    t.tx_buffer = s_line;
    spi_device_transmit(s_tft, &t);
}
void set_scroll_origin(int y) {
    const uint8_t p[] = {(uint8_t)(y >> 8), (uint8_t)y};
    cmd(0x37, p, sizeof(p));
}
}

namespace marauder {
esp_err_t display_bus_init() {
    gpio_config_t cs{};
    cs.pin_bit_mask = 1ULL << pins::tft_cs;
    cs.mode = GPIO_MODE_OUTPUT;
    ESP_RETURN_ON_ERROR(gpio_config(&cs), "display", "chip select gpio");
    gpio_set_level(pins::tft_cs, 1);
    spi_bus_config_t bus{};
    bus.mosi_io_num = pins::tft_mosi;
    // GPIO19 is shared SPI MISO for the SD card; the TFT itself does not use it.
    bus.miso_io_num = pins::spi_miso;
    bus.sclk_io_num = pins::tft_sclk;
    bus.quadwp_io_num = -1; bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 4096;
    esp_err_t e = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    return e == ESP_ERR_INVALID_STATE ? ESP_OK : e;
}

esp_err_t display_init() {
    gpio_config_t outs{};
    outs.pin_bit_mask = (1ULL << pins::tft_dc) | (1ULL << pins::backlight);
    outs.mode = GPIO_MODE_OUTPUT;
    ESP_RETURN_ON_ERROR(gpio_config(&outs), "display", "gpio");
    gpio_set_level(pins::backlight, 0);
    // Marauder v7's TFT reset is tied to board reset (upstream TFT_RST = -1).
    esp_rom_delay_us(120000);
    spi_device_interface_config_t dev{};
    // Classic ESP32 SPI master tops out below 26.67 MHz in this mode.
    dev.clock_speed_hz = 20000000;
    dev.mode = 0;
    dev.spics_io_num = pins::tft_cs;
    dev.queue_size = 2;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &dev, &s_tft), "display", "spi device");
    const uint8_t swreset = 0x01; cmd(swreset); esp_rom_delay_us(150000);
    const uint8_t c0[] = {0x23}; cmd(0xc0, c0, 1);
    const uint8_t c1[] = {0x10}; cmd(0xc1, c1, 1);
    const uint8_t c5[] = {0x3e, 0x28}; cmd(0xc5, c5, 2);
    const uint8_t c7[] = {0x86}; cmd(0xc7, c7, 1);
    const uint8_t madctl[] = {0x48}; cmd(0x36, madctl, 1);
    const uint8_t pixfmt[] = {0x55}; cmd(0x3a, pixfmt, 1);
    const uint8_t b1[] = {0x00, 0x18}; cmd(0xb1, b1, 2);
    const uint8_t b6[] = {0x08, 0x82, 0x27}; cmd(0xb6, b6, 3);
    const uint8_t e0[] = {0x0f,0x31,0x2b,0x0c,0x0e,0x08,0x4e,0xf1,0x37,0x07,0x10,0x03,0x0e,0x09,0x00}; cmd(0xe0, e0, sizeof(e0));
    const uint8_t e1[] = {0x00,0x0e,0x14,0x03,0x11,0x07,0x31,0xc1,0x48,0x08,0x0f,0x0c,0x31,0x36,0x0f}; cmd(0xe1, e1, sizeof(e1));
    cmd(0x11); esp_rom_delay_us(120000); cmd(0x29); esp_rom_delay_us(20000);
    display_clear(kBlack);
    // ILI9341 vertical scrolling: fixed header, scrolling spectrum history.
    const uint8_t scroll_area[] = {0x00, 76, 0x00, 220, 0x00, 24};
    cmd(0x33, scroll_area, sizeof(scroll_area));
    gpio_set_level(pins::backlight, 1);
    return ESP_OK;
}

void display_clear(uint16_t color) {
    spi_gate_lock();
    // The ILI9341 scroll origin is global. Normalize it before drawing menus
    // so stale waterfall rotation can never offset menu rows or leave spots.
    s_scroll = 0;
    set_scroll_origin(kWaterTop);
    fill_rect(0, 0, 239, 319, color);
    spi_gate_unlock();
}

void display_fill_rect(int x, int y, int width, int height, uint16_t color) {
    if (width <= 0 || height <= 0 || x < 0 || y < 0 || x + width > 240 || y + height > 320) return;
    invalidate_text(x, y, width, height);   // cached text under the rectangle is no longer on screen
    spi_gate_lock();
    fill_rect(x, y, x + width - 1, y + height - 1, color);
    spi_gate_unlock();
}

void display_rgb565_image(int x,int y,int width,int height,const uint16_t *pixels){
    if(!pixels||width<=0||width>240||height<=0||x<0||y<0||x+width>240||y+height>320)return;
    invalidate_text(x,y,width,height);
    spi_gate_lock();set_window(x,y,x+width-1,y+height-1);gpio_set_level(pins::tft_dc,1);
    for(int row=0;row<height;++row){for(int col=0;col<width;++col){uint16_t v=pixels[row*width+col];s_line[col*2]=(uint8_t)(v>>8);s_line[col*2+1]=(uint8_t)v;}spi_transaction_t t{};t.length=(size_t)width*16;t.tx_buffer=s_line;spi_device_transmit(s_tft,&t);}
    spi_gate_unlock();
}

void display_text(int x, int y, const char *s, uint16_t fg, uint16_t bg) {
    if (!s || x < 0 || y < 0 || y + 8 > 320 || x >= 240) return;
    size_t count = 0;
    while (s[count] && x + (int)(count + 1) * 8 <= 240) ++count;
    if (!count) return;
    char clipped[31];
    if (count > sizeof(clipped)-1) count=sizeof(clipped)-1;
    memcpy(clipped,s,count);clipped[count]=0;
    for (auto &e : s_text_cache) {
        if (e.valid && e.x==x && e.y==y && e.fg==fg && e.bg==bg && strcmp(e.text,clipped)==0) return;
    }
    TextCache *slot=nullptr;
    for (auto &e : s_text_cache) if (e.valid && e.x==x && e.y==y) { slot=&e; break; }
    if(!slot){slot=&s_text_cache[s_text_cache_next++%32];}
    const size_t previous_count=slot->valid&&slot->x==x&&slot->y==y?strlen(slot->text):0;
    const size_t draw_count=std::max(count,previous_count);
    slot->x=x;slot->y=y;slot->fg=fg;slot->bg=bg;strcpy(slot->text,clipped);slot->valid=true;
    spi_gate_lock();
    for (int row = 0; row < 8; ++row) {
        for (size_t ch = 0; ch < draw_count; ++ch) {
            uint8_t bits = ch<count?font::glyphs[(uint8_t)s[ch]][row]:font::glyphs[(uint8_t)' '][row];
            for (int col = 0; col < 8; ++col) {
                uint16_t c = (bits & (0x80u >> col)) ? fg : bg;
                size_t p = (ch * 8 + col) * 2;
                s_text_line[p] = (uint8_t)(c >> 8);
                s_text_line[p + 1] = (uint8_t)c;
            }
        }
        set_window(x, y + row, x + (int)draw_count * 8 - 1, y + row);
        gpio_set_level(pins::tft_dc, 1);
        spi_transaction_t t{};
        t.length = draw_count * 8 * 16;
        t.tx_buffer = s_text_line;
        spi_device_transmit(s_tft, &t);
    }
    spi_gate_unlock();
}

void display_waterfall_line(const uint16_t *colors) {
    spi_gate_lock();
    s_scroll = (s_scroll + 1) % kWaterHeight;
    // Advance the viewport first, then write the new sample into the bottom
    // row of that viewport. Writing against the old origin leaves stale rows
    // behind at each wrap of the circular waterfall.
    int physical = kWaterTop + ((s_scroll + kWaterHeight - 1) % kWaterHeight);
    push_scanline(physical, colors);
    uint16_t start = (uint16_t)(kWaterTop + s_scroll);
    const uint8_t p[] = {(uint8_t)(start >> 8), (uint8_t)start};
    cmd(0x37, p, sizeof(p));
    spi_gate_unlock();
}

void display_reset_waterfall() {
    spi_gate_lock();
    s_scroll = 0;
    set_scroll_origin(kWaterTop);
    fill_rect(0, kWaterTop, 239, kWaterTop + kWaterHeight - 1, kBlack);
    spi_gate_unlock();
}

void display_spectrum(const uint8_t *bins, const uint8_t *peaks, uint16_t color) {
    display_spectrum_region(bins, peaks, color, 36, 28);
}

void display_spectrum_region(const uint8_t *bins, const uint8_t *peaks, uint16_t color,
                             int top, int height, const uint8_t *floor_marks, const uint8_t *min_marks) {
    if (height < 2) return;
    spi_gate_lock();
    uint16_t row[240];
    int trace_y[240], peak_y[240], floor_y[240], min_y[240];
    const int plot_height = height - 4;
    for (int x = 0; x < 240; ++x) {
        trace_y[x] = height - 1 - (bins[x] * plot_height) / 255;
        peak_y[x] = height - 1 - (peaks[x] * plot_height) / 255;
        floor_y[x] = floor_marks ? height - 1 - (floor_marks[x] * plot_height) / 255 : -1;
        min_y[x] = min_marks ? height - 1 - (min_marks[x] * plot_height) / 255 : -1;
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < 240; ++x) {
            const int above = x > 0 ? x - 1 : x;
            const bool has_signal = bins[above] || bins[x];
            const bool trace = has_signal &&
                               y >= std::min(trace_y[above], trace_y[x]) - 1 &&
                               y <= std::max(trace_y[above], trace_y[x]);
            const bool peak = peaks[x] > bins[x] && y == peak_y[x];
            row[x] = trace ? color : peak ? 0xf800 : (min_y[x] == y) ? 0x07e0 : (floor_y[x] == y && (x & 1) == 0) ? 0xfd20 : 0x0000;
        }
        push_scanline(top + y, row);
    }
    spi_gate_unlock();
}

}
