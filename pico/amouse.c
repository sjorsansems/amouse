/*
 *   __ _   _ __  ___ _  _ ___ ___ 
 *  / _` | | '  \/ _ \ || (_-</ -_)
 *  \__,_| |_|_|_\___/\_,_/__/\___=====_____)
 *
 * Anachro Mouse, a usb to serial mouse adaptor. Copyright (C) 2021-2025 Aviancer <oss+amouse@skyvian.me>
 *
 * This library is free software; you can redistribute it and/or modify it under the terms of the 
 * GNU Lesser General Public License as published by the Free Software Foundation; either version 
 * 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without 
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this library; 
 * if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
*/

#include <time.h>
#include "stdbool.h"
#include <stdio.h>
#include "pico/flash.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/irq.h"

#include "include/version.h"
#include "include/serial.h"
#include "include/storage.h"
#include "include/usb.h"
#include "include/oled.h"
#include "../shared/console.h"
#include "../shared/utils.h"
#include "../shared/mouse.h"
#include "../shared/mouse_defs.h"
#include "../shared/settings.h"

#include "bsp/board.h"
#include "tusb.h"

static const uint8_t bitmap10[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x3e, 0x00, 0x00,
    0x3f, 0xc0, 0x00, 0x1f, 0xf0, 0x00, 0x1c, 0xfc, 0x00, 0x0c, 0x3f, 0x80,
    0x0e, 0x07, 0xe0, 0x0e, 0x01, 0xf8, 0x06, 0x01, 0xf8, 0x07, 0x0f, 0xf0,
    0x03, 0x1f, 0x80, 0x03, 0x9e, 0x00, 0x03, 0x9f, 0x00, 0x01, 0xfb, 0x80,
    0x01, 0xf9, 0xc0, 0x00, 0xf0, 0xe0, 0x00, 0xf0, 0x70, 0x00, 0x70, 0x30,
    0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t bitmap12[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x80, 0x3f, 0xff, 0xc0,
    0x33, 0x00, 0xe0, 0x33, 0x00, 0x70, 0x33, 0x00, 0x38, 0x33, 0xff, 0x1c,
    0x33, 0xff, 0x0c, 0x30, 0x00, 0x0c, 0x30, 0x00, 0x0c, 0x30, 0x00, 0x0c,
    0x33, 0xff, 0xcc, 0x33, 0xff, 0xcc, 0x33, 0x00, 0xcc, 0x33, 0x00, 0xcc,
    0x33, 0x00, 0xcc, 0x33, 0x00, 0xcc, 0x33, 0x00, 0xcc, 0x33, 0x00, 0xdc,
    0x3f, 0xff, 0xfc, 0x1f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t bitmap14[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00,
    0x01, 0xe0, 0x00, 0x01, 0xe0, 0x00, 0x01, 0xe0, 0x00, 0x03, 0xf0, 0x00,
    0x03, 0xf0, 0x00, 0x03, 0xf0, 0x00, 0x07, 0x38, 0x00, 0x07, 0x38, 0x00,
    0x7f, 0x38, 0x7e, 0x7e, 0x1c, 0xfe, 0x00, 0x1c, 0xe0, 0x00, 0x1c, 0xe0,
    0x00, 0x0f, 0xc0, 0x00, 0x0f, 0xc0, 0x00, 0x0f, 0xc0, 0x00, 0x07, 0x80,
    0x00, 0x07, 0x80, 0x00, 0x07, 0x80, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00
};

/*** Mouse state variables ****/
mouse_state_t mouse;

static uint32_t time_tx_target;  
static uint32_t time_rx_target;  

uint8_t serial_buffer[2] = {0};

// ===== PROTOCOL BUTTON CONFIG =====
const uint PROTOCOL_BUTTON_PIN = 9;
uint32_t protocol_button_timer = 0;
bool protocol_change_message_active = false;
bool protocol_button_was_pressed = false;
bool protocol_button_longpress_handled = false;
uint32_t protocol_button_press_start = 0;
uint32_t status_message_timer = 0;
uint8_t protocol_button_click_count = 0;
uint32_t protocol_button_click_window_start = 0;
bool speed_adjust_mode_active = false;
uint32_t speed_adjust_last_activity = 0;
bool diagnostic_mode_active = false;
uint32_t last_hid_activity_us = 0;

#define PROTOCOL_BUTTON_DEBOUNCE_US 50000
#define PROTOCOL_BUTTON_SAVE_US 5000000
#define PROTOCOL_BUTTON_CLICK_WINDOW_US 350000
#define STATUS_MESSAGE_DURATION_US 2000000
#define SPEED_ADJUST_TIMEOUT_US 10000000
#define DATA_ACTIVITY_TIMEOUT_US 2000000
#define CTS_LOW_STABLE_US 2000
#define CTS_REIDENT_COOLDOWN_US 1500000
#define CTS_BOOTSTRAP_LOW_US 1500000

CFG_TUSB_MEM_SECTION static hid_mouse_report_t usb_mouse_report_prev;

// ===== LED CONFIG =====
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
bool led_state = false;
uint32_t led_timer = 0;
uint32_t cts_low_since = 0;
uint32_t cts_boot_started = 0;
uint32_t cts_last_ident = 0;

/*** Timing ***/
static void save_current_settings(void) {
    uint8_t binary_settings[SETTINGS_SIZE] = {0};
    settings_encode(&binary_settings[0], &g_mouse_options);
    write_flash_settings(&binary_settings[0], sizeof(binary_settings));
}

static void show_status_message(const char *message, uint32_t now) {
    protocol_change_message_active = true;
    status_message_timer = now;
    oled_clear();
    oled_write_text(message, 1);
}

static void show_saving_message(uint32_t now) {
    protocol_change_message_active = true;
    status_message_timer = now;
    oled_clear();
    oled_draw_bitmap_mono(0, 4, bitmap12, 24, 24);
    oled_write_text("", 0);
    oled_write_text_at("SAVING..", 1, 32);
    oled_write_text_at("OK", 2, 32);
    oled_write_text("", 3);
}

static void show_speed_adjust_screen(uint32_t now) {
    char speed_text[16];
    char exit_text[16];
    int speed_value = clampi((int)(g_mouse_options.sensitivity * 10.0f + 0.5f), 2, 30);
    uint32_t elapsed_us = now - speed_adjust_last_activity;
    int exit_in = (elapsed_us >= SPEED_ADJUST_TIMEOUT_US) ? 0 : (int)((SPEED_ADJUST_TIMEOUT_US - elapsed_us + 999999) / 1000000);
    oled_clear();
    oled_write_text("", 0);
    oled_write_text("SET SPEED: 2-30", 1);
    snprintf(speed_text, sizeof(speed_text), "NOW: %d", speed_value);
    oled_write_text(speed_text, 2);
    snprintf(exit_text, sizeof(exit_text), "EXIT IN: %d", exit_in);
    oled_write_text(exit_text, 3);
}

static void show_diagnostic_screen(uint32_t now, bool cts_pin) {
    char line0[20];
    char line1[20];
    char line2[20];
    char line3[20];
    bool data_active = g_usb_device_connected && ((now - last_hid_activity_us) < DATA_ACTIVITY_TIMEOUT_US);
    uint32_t uptime_s = to_ms_since_boot(get_absolute_time()) / 1000;

    oled_clear();
    snprintf(line0, sizeof(line0), "DATA: %s", data_active ? "OK" : "IDLE");
    snprintf(line1, sizeof(line1), "CTS: %s", cts_pin ? "HIGH" : "LOW");
    snprintf(line2, sizeof(line2), "UP: %lus", (unsigned long)uptime_s);
    snprintf(line3, sizeof(line3), "FW: %s", V_FULL);
    oled_draw_bitmap_mono(0, 4, bitmap14, 24, 24);
    oled_write_text("", 0);
    oled_write_text_at(line0, 0, 32);
    oled_write_text_at(line1, 1, 32);
    oled_write_text_at(line2, 2, 32);
    oled_write_text_at(line3, 3, 32);
}

void queue_tx(mouse_state_t *mouse) {
    if(mouse->update > 3) { time_tx_target = time_us_32() + U_SERIALDELAY_4B; }
    else                  { time_tx_target = time_us_32() + U_SERIALDELAY_3B; }
}

/*** Mouse USB handling ***/
int test_mouse_button(uint8_t buttons_state, uint8_t button) {
    return (buttons_state & button) ? 1 : 0;
}

static inline void process_mouse_report(mouse_state_t *mouse, hid_mouse_report_t const *p_report) {
    uint8_t button_changed_mask = p_report->buttons ^ usb_mouse_report_prev.buttons;
    if(button_changed_mask) {
        mouse->force_update = 1;
        mouse->lmb = test_mouse_button(p_report->buttons, MOUSE_BUTTON_LEFT);
        mouse->rmb = test_mouse_button(p_report->buttons, MOUSE_BUTTON_RIGHT);
        push_update(mouse, mouse->mmb);

        if(button_changed_mask & MOUSE_BUTTON_MIDDLE) {
            mouse->mmb = test_mouse_button(p_report->buttons, MOUSE_BUTTON_MIDDLE);
            if(g_mouse_protocol[g_mouse_options.protocol].buttons > 2) {
                push_update(mouse, true); 
            }
        }
    }

    if(p_report->x) {
        mouse->x += p_report->x;
        mouse->x = clampi(mouse->x, -36862, 36862); 
        push_update(mouse, mouse->mmb);
    }
    if(p_report->y) {
        mouse->y += p_report->y;
        mouse->y = clampi(mouse->y, -36862, 36862);
        push_update(mouse, mouse->mmb);
    }
    if(p_report->wheel) {
        mouse->wheel += p_report->wheel;
        mouse->wheel = clampi(mouse->wheel, -63, 63);
        if(g_mouse_protocol[g_mouse_options.protocol].wheel) {
            push_update(mouse, true); 
        }
    }

    usb_mouse_report_prev = *p_report;
}

extern void collect_mouse_report(hid_mouse_report_t const* p_report) {
    process_mouse_report(&mouse, p_report);
    last_hid_activity_us = time_us_32();
}

/*** Core 1 thread ***/
void core1_tightloop() {
    flash_safe_execute_core_init();
    uint8_t serial_data;
    while(1) {
        queue_remove_blocking(&g_serial_queue, &serial_data);
        uart_putc_raw(uart0, serial_data);
    }
}

/*** Main init & loop ***/
int main() {

    mouse_serial_init(0); 
    queue_init(&g_serial_queue, sizeof(uint8_t), 80);
    multicore_launch_core1(core1_tightloop);

    reset_mouse_state(&mouse);
    mouse.pc_state = CTS_UNINIT;

    g_mouse_options.protocol = PROTO_MSWHEEL;
    g_mouse_options.wheel = 1;
    g_mouse_options.sensitivity = 0.2;
    settings_decode(ptr_flash_settings(), &g_mouse_options);

    tusb_init();

    // ===== LED INIT =====
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    led_timer = time_us_32();

    // ===== PROTOCOL BUTTON INIT =====
    gpio_init(PROTOCOL_BUTTON_PIN);
    gpio_set_dir(PROTOCOL_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(PROTOCOL_BUTTON_PIN);

    // ===== OLED INIT =====
    oled_init();
    oled_clear();
    char fw_intro[16];
    snprintf(fw_intro, sizeof(fw_intro), "FW: %s", V_FULL);
    oled_draw_bitmap_mono(0, 4, bitmap10, 24, 24);
    oled_write_text("", 0);
    oled_write_text_at("RETRO GEORGY", 1, 32);
    oled_write_text_at("SERIAL 2 USB", 2, 32);
    oled_write_text_at(fw_intro, 3, 32);
    sleep_ms(8000);
    oled_clear();

    time_tx_target = time_us_32() + U_SERIALDELAY_3B;
    time_rx_target = time_us_32() + U_FULL_SECOND; 

    bool cts_pin = false;
    uint32_t oled_update_timer = time_us_32();
    cts_boot_started = time_us_32();

    while(1) {
        uint32_t now = time_us_32();

        // ===== SERIAL CONSOLE =====
        if(time_reached(time_rx_target)) {
            if(serial_read(0, serial_buffer, 1) > 0) {
                if(serial_buffer[0] == '\b') {
                    console(0);
                }
            }
            time_rx_target = time_us_32() + U_FULL_SECOND;
        }

        // ===== CTS / Mouse Handling =====
        cts_pin = gpio_get(UART_CTS_PIN);

        if(cts_pin) {
            cts_low_since = 0;
            if(mouse.pc_state == CTS_UNINIT) mouse.pc_state = CTS_LOW_INIT;
            else if(mouse.pc_state == CTS_TOGGLED) mouse.pc_state = CTS_LOW_RUN;
        }

        if(!cts_pin) {
            if(cts_low_since == 0) {
                cts_low_since = now;
            }

            bool cts_low_stable = (now - cts_low_since) > CTS_LOW_STABLE_US;
            bool cts_handshake_edge = (mouse.pc_state != CTS_UNINIT && mouse.pc_state != CTS_TOGGLED);
            bool cts_bootstrap = (mouse.pc_state == CTS_UNINIT) && ((now - cts_boot_started) > CTS_BOOTSTRAP_LOW_US);
            bool cts_ident_cooldown_ok = (now - cts_last_ident) > CTS_REIDENT_COOLDOWN_US;

            // Handle both normal CTS edge negotiation and restart-while-CTS-low cases.
            if(cts_low_stable && cts_ident_cooldown_ok && (cts_handshake_edge || cts_bootstrap)) {
                gpio_put(LED_PIN, 0);
                sleep_ms(50);
                gpio_put(LED_PIN, 1);
                mouse.pc_state = CTS_TOGGLED;
                cts_last_ident = now;
                show_status_message("PC found mouse", now);
                mouse_ident(0, g_mouse_options.wheel);
            }
        }

        if(mouse.pc_state > CTS_LOW_INIT) {
            tuh_task();
            if(time_reached(time_tx_target) || mouse.force_update) {
                runtime_settings(&mouse);
                input_sensitivity(&mouse);
                update_mouse_state(&mouse);
                queue_tx(&mouse);
                if(mouse.update > 0) serial_write(0, mouse.state, mouse.update);
                reset_mouse_state(&mouse);
            }
        }

        // ===== LED STATUS =====
        if(mouse.pc_state > CTS_LOW_INIT) {
            gpio_put(LED_PIN, 1);
        } else if (time_us_32() - led_timer > 500000) {
            led_state = !led_state;
            gpio_put(LED_PIN, led_state);
            led_timer = time_us_32();
        }

        // ===== PROTOCOL BUTTON =====
        bool protocol_button_pressed = (gpio_get(PROTOCOL_BUTTON_PIN) == 0);
        if(protocol_button_pressed && !protocol_button_was_pressed && now - protocol_button_timer > PROTOCOL_BUTTON_DEBOUNCE_US) {
            protocol_button_timer = now;
            protocol_button_was_pressed = true;
            protocol_button_longpress_handled = false;
            protocol_button_press_start = now;
        }

        if(protocol_button_pressed && protocol_button_was_pressed && !protocol_button_longpress_handled) {
            if(now - protocol_button_press_start >= PROTOCOL_BUTTON_SAVE_US) {
                protocol_button_longpress_handled = true;
                protocol_button_click_count = 0;
                speed_adjust_mode_active = false;
                diagnostic_mode_active = false;
                save_current_settings();

                show_saving_message(now);
            }
        }

        if(!protocol_button_pressed && protocol_button_was_pressed && now - protocol_button_timer > PROTOCOL_BUTTON_DEBOUNCE_US) {
            protocol_button_timer = now;
            protocol_button_was_pressed = false;

            if(!protocol_button_longpress_handled) {
                if(speed_adjust_mode_active) {
                    int speed_value = clampi((int)(g_mouse_options.sensitivity * 10.0f + 0.5f), 2, 30);
                    speed_value++;
                    if(speed_value > 30) speed_value = 2;
                    g_mouse_options.sensitivity = speed_value / 10.0f;
                    speed_adjust_last_activity = now;
                    show_speed_adjust_screen(now);
                } else {
                    if(protocol_button_click_count == 0 || (now - protocol_button_click_window_start > PROTOCOL_BUTTON_CLICK_WINDOW_US)) {
                        protocol_button_click_count = 1;
                    } else {
                        protocol_button_click_count++;
                    }
                    protocol_button_click_window_start = now;
                }
            }
        }

        if(!speed_adjust_mode_active && protocol_button_click_count > 0 && (now - protocol_button_click_window_start > PROTOCOL_BUTTON_CLICK_WINDOW_US)) {
            if(protocol_button_click_count == 1) {
                diagnostic_mode_active = false;
                g_mouse_options.protocol = (g_mouse_options.protocol + 1) % 3;
            } else if(protocol_button_click_count == 2) {
                diagnostic_mode_active = false;
                speed_adjust_mode_active = true;
                speed_adjust_last_activity = now;
                show_speed_adjust_screen(now);
            } else {
                speed_adjust_mode_active = false;
                diagnostic_mode_active = !diagnostic_mode_active;
                if(diagnostic_mode_active) {
                    show_diagnostic_screen(now, cts_pin);
                } else {
                    oled_clear();
                }
            }

            protocol_button_click_count = 0;
            oled_update_timer = 0;
        }

        if(speed_adjust_mode_active && (now - speed_adjust_last_activity > SPEED_ADJUST_TIMEOUT_US)) {
            speed_adjust_mode_active = false;
            oled_clear();
            oled_update_timer = 0;
        }

        if(protocol_change_message_active && (now - status_message_timer > STATUS_MESSAGE_DURATION_US)) {
            protocol_change_message_active = false;
        }

        // ===== OLED STATUS =====
        if(time_us_32() - oled_update_timer > 1000000) { // Update elke seconde
            oled_update_timer = time_us_32();
            if(!protocol_change_message_active) {
                if(speed_adjust_mode_active) {
                    show_speed_adjust_screen(now);
                } else if(diagnostic_mode_active) {
                    show_diagnostic_screen(now, cts_pin);
                } else {
                char usb_text[16];
                snprintf(usb_text, sizeof(usb_text), "USB:    %s", g_usb_device_connected ? "OK" : "NOT OK");
                oled_write_text(usb_text, 0);
                if((int32_t)(g_usb_status_message_until - now) > 0) {
                    oled_write_text(g_usb_status_message, 1);
                }
                else {
                    char serial_text[16];
                    snprintf(serial_text, sizeof(serial_text), "SERIAL: %s", !cts_pin ? "OK" : "NOT OK");
                    oled_write_text(serial_text, 1);
                }
                char speed_text[16];
                snprintf(speed_text, sizeof(speed_text), "SPEED:  %d", (int)(g_mouse_options.sensitivity * 10));
                oled_write_text(speed_text, 2);
                char type_text[32];
                snprintf(type_text, sizeof(type_text), "TYPE:   %s", g_mouse_protocol[g_mouse_options.protocol].name);
                oled_write_text(type_text, 3);
                }
            }
        }
    }

    return 0;
}