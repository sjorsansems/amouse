/*
 * OLED header for SSD1306.
 */

#ifndef OLED_H_
#define OLED_H_

#include "pico/stdlib.h"

void oled_init();
void oled_clear();
void oled_write_text(const char* text, uint8_t line);
void oled_write_text_at(const char* text, uint8_t line, uint8_t col);
void oled_draw_bitmap_mono(uint8_t x, uint8_t y, const uint8_t* bitmap, uint8_t width, uint8_t height);

#endif