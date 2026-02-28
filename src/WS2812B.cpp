/**
 * ESP32_WS2812B - A simple library for controlling WS2812B LEDs using ESP32's RMT peripheral.
 * 
 * Copyright (c) 2026 Xorlent
 * Licensed under the MIT License.
 * https://github.com/Xorlent/ESP32_WS2812B
 * 
 */

#include "WS2812B.h"
#include <driver/rmt_tx.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>

// WS2812B timing (in RMT ticks, 1 tick = 12.5ns at 80MHz)
#define WS2812B_T0H_TICKS 24    // 0 code, high level time (300ns / 12.5ns ≈ 24)
#define WS2812B_T0L_TICKS 80    // 0 code, low level time (1000ns / 12.5ns ≈ 80)
#define WS2812B_T1H_TICKS 80   // 1 code, high level time (1000ns / 12.5ns ≈ 80)
#define WS2812B_T1L_TICKS 24    // 1 code, low level time (300ns / 12.5ns ≈ 24)

// Custom RMT encoder for WS2812B
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t reset_code;
    int state;
} rmt_led_strip_encoder_t;

static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                   const void *primary_data, size_t data_size,
                                   rmt_encode_state_t *ret_state) {
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    int state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    
    // Safety check
    if (!bytes_encoder || !copy_encoder || !primary_data || !ret_state) {
        if (ret_state) {
            *ret_state = RMT_ENCODING_COMPLETE;
        }
        return 0;
    }
    
    switch (led_encoder->state) {
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                               sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }
out:
    *ret_state = (rmt_encode_state_t)state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder) {
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    if (led_encoder->bytes_encoder) {
        rmt_del_encoder(led_encoder->bytes_encoder);
    }
    if (led_encoder->copy_encoder) {
        rmt_del_encoder(led_encoder->copy_encoder);
    }
    delete led_encoder;
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder) {
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    if (led_encoder->bytes_encoder) {
        rmt_encoder_reset(led_encoder->bytes_encoder);
    }
    if (led_encoder->copy_encoder) {
        rmt_encoder_reset(led_encoder->copy_encoder);
    }
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder) {
    rmt_led_strip_encoder_t *led_encoder = new rmt_led_strip_encoder_t();
    if (!led_encoder) {
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize pointers to nullptr for safe cleanup
    led_encoder->bytes_encoder = nullptr;
    led_encoder->copy_encoder = nullptr;
    
    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;
    
    // Create bytes encoder with WS2812B timing
    rmt_bytes_encoder_config_t bytes_encoder_config = {};
    bytes_encoder_config.bit0.level0 = 1;
    bytes_encoder_config.bit0.duration0 = WS2812B_T0H_TICKS;
    bytes_encoder_config.bit0.level1 = 0;
    bytes_encoder_config.bit0.duration1 = WS2812B_T0L_TICKS;
    bytes_encoder_config.bit1.level0 = 1;
    bytes_encoder_config.bit1.duration0 = WS2812B_T1H_TICKS;
    bytes_encoder_config.bit1.level1 = 0;
    bytes_encoder_config.bit1.duration1 = WS2812B_T1L_TICKS;
    bytes_encoder_config.flags.msb_first = 1; // WS2812B sends MSB first
    
    if (rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder) != ESP_OK) {
        delete led_encoder;
        return ESP_FAIL;
    }
    
    // Create copy encoder for reset code
    rmt_copy_encoder_config_t copy_encoder_config = {};
    if (rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder) != ESP_OK) {
        rmt_del_encoder(led_encoder->bytes_encoder);
        delete led_encoder;
        return ESP_FAIL;
    }
    
    // WS2812B reset code (minimal - actual holdoff handled in sendData)
    led_encoder->reset_code.level0 = 0;
    led_encoder->reset_code.duration0 = 80; // Minimal reset pulse
    led_encoder->reset_code.level1 = 0;
    led_encoder->reset_code.duration1 = 0;
    led_encoder->state = RMT_ENCODING_RESET;
    
    *ret_encoder = &led_encoder->base;
    return ESP_OK;
}

WS2812B::WS2812B() : rmt_channel(nullptr), led_encoder(nullptr), initialized(false), last_transmit_us(0), tx_config{} {
}

WS2812B::~WS2812B() {
    if (initialized) {
        if (rmt_channel) {
            rmt_disable(rmt_channel);
        }
        if (led_encoder) {
            rmt_del_encoder(led_encoder);
            led_encoder = nullptr;
        }
        if (rmt_channel) {
            rmt_del_channel(rmt_channel);
            rmt_channel = nullptr;
        }
        initialized = false;
    }
}

bool WS2812B::begin(uint8_t pin) {
    // Clean up any existing resources if begin() called multiple times
    if (initialized) {
        if (rmt_channel) {
            rmt_disable(rmt_channel);
        }
        if (led_encoder) {
            rmt_del_encoder(led_encoder);
            led_encoder = nullptr;
        }
        if (rmt_channel) {
            rmt_del_channel(rmt_channel);
            rmt_channel = nullptr;
        }
        initialized = false;
    }
    
    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_chan_config = {};
    tx_chan_config.gpio_num = (gpio_num_t)pin;
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_config.resolution_hz = 80000000; // 80MHz resolution = 12.5ns per tick
    tx_chan_config.mem_block_symbols = 64;
    tx_chan_config.trans_queue_depth = 1;
    
    if (rmt_new_tx_channel(&tx_chan_config, &rmt_channel) != ESP_OK) {
        rmt_channel = nullptr;
        return false;
    }
    
    // Create LED strip encoder
    if (rmt_new_led_strip_encoder(&led_encoder) != ESP_OK) {
        rmt_del_channel(rmt_channel);
        rmt_channel = nullptr;
        led_encoder = nullptr;
        return false;
    }
    
    // Enable RMT channel
    if (rmt_enable(rmt_channel) != ESP_OK) {
        rmt_del_encoder(led_encoder);
        rmt_del_channel(rmt_channel);
        rmt_channel = nullptr;
        led_encoder = nullptr;
        return false;
    }
    
    initialized = true;
    
    // Initialize to off
    set("black", 0);
    
    return true;
}

void WS2812B::set(const char* color, uint8_t brightness) {
    if (!initialized || !color) {
        return;
    }
    
    uint8_t r = 0, g = 0, b = 0;
    
    // Use character checking for efficiency
    char first = color[0];
    
    if (first == 'r' || first == 'R') {  // "red" or "R"
        r = 255u;
    } else if (first == 'g' || first == 'G') {  // "green" or "G"
        g = 255u;
    } else if (first == 'B' || (color[1] == 'l' && color[2] == 'u')) {  // "B" or "blue"
        b = 255u;
    } else if (first == 'p') {  // "purple"
        r = 128u;
        b = 128u;
    } else if (first == 'y') {  // "yellow"
        r = 255u;
        g = 150u;
    } else if (first == 'o') {  // "orange"
        r = 255u;
        g = 75u;
    } else if (first == 'w') {  // "white"
        r = g = b = 255u;
    } else {
        // Unrecognized color or black
        sendData(0u, 0u, 0u);
        return;
    }
    
    // Apply brightness scaling
    if (brightness == 0u) {
        sendData(0, 0, 0);  // Skip math entirely
        return;
    } else if (brightness < 255u) {
        r = ((r * brightness) + 128) >> 8;
        g = ((g * brightness) + 128) >> 8;
        b = ((b * brightness) + 128) >> 8;
    }
    
    sendData(r, g, b);
}

void WS2812B::sendData(uint8_t r, uint8_t g, uint8_t b) {
    if (!rmt_channel || !led_encoder) {
        return;
    }
    
    // WS2812B requires 250us holdoff between transmissions
    // Only wait if we're transmitting again before the holdoff period completes
    const int64_t HOLDOFF_US = 250;
    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - last_transmit_us;
    
    if (last_transmit_us > 0 && elapsed_us >= 0 && elapsed_us < HOLDOFF_US) {
        // Wait only the remaining required time
        int64_t remaining_us = HOLDOFF_US - elapsed_us;
        esp_rom_delay_us((uint32_t)remaining_us);
    }
    
    // WS2812B expects GRB order
    uint8_t led_data[3] = {g, r, b};
    
    // Transmit data via RMT (blocks until queued, copies data internally)
    rmt_transmit(rmt_channel, led_encoder, led_data, sizeof(led_data), &tx_config);
    
    // Wait for transmission to complete
    rmt_tx_wait_all_done(rmt_channel, 1);
    
    // Store completion timestamp for next transmission
    last_transmit_us = esp_timer_get_time();
}
