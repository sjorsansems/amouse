/*
 * Anachro Mouse OLED driver for SSD1306 0.91" I2C display.
 * Based on open-source SSD1306 drivers.
 */

#include "oled.h"
#include "hardware/i2c.h"
#include <string.h>

// I2C config
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5
#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 32
#define OLED_PAGES (OLED_HEIGHT / 8)

// Font: 5x7 pixels, voor ASCII 32-126 (vereenvoudigd, alleen hoofdletters en spaties)
static const uint8_t font5x7[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32 space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33 !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34 "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35 #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36 $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37 %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38 &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39 '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 40 (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 41 )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // 42 *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43 +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 44 ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45 -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46 .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47 /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58 :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59 ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 60 <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61 =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 62 >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63 ?
    {0x32, 0x49, 0x59, 0x51, 0x3E}, // 64 @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 65 A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 66 B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 67 C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 68 D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 69 E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 70 F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 71 G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 72 H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 73 I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 74 J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 75 K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 76 L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 77 M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 78 N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 79 O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 80 P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 81 Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 82 R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83 S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 84 T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 85 U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 86 V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 87 W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88 X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 89 Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90 Z
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 91 [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92 \
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 93 ]
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 94 ^
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 95 _
};

static uint8_t buffer[OLED_WIDTH * OLED_PAGES];

void oled_send_command(uint8_t cmd);
void oled_send_data(uint8_t *data, size_t len);

static void oled_refresh_pages(uint8_t start_page, uint8_t end_page) {
    if (start_page >= OLED_PAGES) return;
    if (end_page >= OLED_PAGES) end_page = OLED_PAGES - 1;

    oled_send_command(0x21);
    oled_send_command(0);
    oled_send_command(127);
    oled_send_command(0x22);
    oled_send_command(start_page);
    oled_send_command(end_page);
    oled_send_data(&buffer[start_page * OLED_WIDTH], (end_page - start_page + 1) * OLED_WIDTH);
}

static void oled_set_pixel(uint8_t x, uint8_t y) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    buffer[(y / 8) * OLED_WIDTH + x] |= (1u << (y % 8));
}

void oled_send_command(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};
    i2c_write_blocking(I2C_PORT, OLED_ADDR, data, 2, false);
}

void oled_send_data(uint8_t *data, size_t len) {
    const size_t CHUNK_SIZE = 32;
    uint8_t buf[CHUNK_SIZE + 1];

    while (len > 0) {
        size_t chunk = len;
        if (chunk > CHUNK_SIZE) chunk = CHUNK_SIZE;
        buf[0] = 0x40;
        memcpy(&buf[1], data, chunk);
        i2c_write_blocking(I2C_PORT, OLED_ADDR, buf, chunk + 1, false);
        data += chunk;
        len -= chunk;
    }
}

void oled_init() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // SSD1306 init commands
    uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x02, 0xA0, 0xC0, 0xDA, 0x02,
        0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        oled_send_command(init_cmds[i]);
    }
    oled_clear();
}

void oled_clear() {
    memset(buffer, 0, sizeof(buffer));
    oled_refresh_pages(0, OLED_PAGES - 1);
}

void oled_write_text(const char* text, uint8_t line) {
    if (line >= OLED_PAGES) return;
    uint8_t *buf = &buffer[line * OLED_WIDTH];
    memset(buf, 0, OLED_WIDTH);
    size_t len = strlen(text);
    for (size_t i = 0; i < len && i * 6 < OLED_WIDTH; i++) {
        char c = text[i];
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c < 32 || c > 126) c = 32;
        const uint8_t *glyph = font5x7[c - 32];
        for (int j = 0; j < 5; j++) {
            buf[i * 6 + j] = glyph[j];
        }
        // space tussen chars
    }
    // Update display
    oled_send_command(0x21);
    oled_send_command(0);
    oled_send_command(127);
    oled_send_command(0x22);
    oled_send_command(line);
    oled_send_command(line);
    oled_send_data(buf, OLED_WIDTH);
}

void oled_write_text_at(const char* text, uint8_t line, uint8_t col) {
    if (line >= OLED_PAGES || col >= OLED_WIDTH) return;

    uint8_t *buf = &buffer[line * OLED_WIDTH];
    size_t len = strlen(text);
    for (size_t i = 0; i < len; i++) {
        uint16_t x = col + (uint16_t)(i * 6);
        if (x + 5 >= OLED_WIDTH) break;

        char c = text[i];
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c < 32 || c > 126) c = 32;
        const uint8_t *glyph = font5x7[c - 32];
        for (int j = 0; j < 5; j++) {
            buf[x + j] = glyph[j];
        }
        buf[x + 5] = 0x00;
    }

    oled_send_command(0x21);
    oled_send_command(0);
    oled_send_command(127);
    oled_send_command(0x22);
    oled_send_command(line);
    oled_send_command(line);
    oled_send_data(buf, OLED_WIDTH);
}

void oled_draw_bitmap_mono(uint8_t x, uint8_t y, const uint8_t* bitmap, uint8_t width, uint8_t height) {
    if (bitmap == NULL || x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    uint16_t bytes_per_row = (width + 7) / 8;
    for (uint8_t row = 0; row < height; row++) {
        for (uint8_t col = 0; col < width; col++) {
            uint16_t byte_index = row * bytes_per_row + (col / 8);
            uint8_t bit_mask = 0x80 >> (col % 8);
            if (bitmap[byte_index] & bit_mask) {
                oled_set_pixel(x + col, y + row);
            }
        }
    }

    oled_refresh_pages(y / 8, (y + height - 1) / 8);
}