/**
 * ESP32_WS2812B - A simple library for controlling WS2812B LED strips using ESP32's RMT peripheral.
 * 
 * Copyright (c) 2026 Xorlent
 * Licensed under the MIT License.
 * https://github.com/Xorlent/ESP32_WS2812B
 * 
 */

#ifndef WS2812B_H
#define WS2812B_H

#include <stdint.h>
#include <driver/rmt_tx.h>

class WS2812B {
public:
    WS2812B();
    ~WS2812B();
    
    // Initialize with GPIO pin number
    bool begin(uint8_t pin);
    
    // Set color with brightness (1-255, 255 = full)
    // Colors: "black", "white", "red", "green", "blue", "purple", "yellow", "orange"
    void set(const char* color, uint8_t brightness = 255);
    
private:
    rmt_channel_handle_t rmt_channel;
    rmt_encoder_handle_t led_encoder;
    rmt_transmit_config_t tx_config;
    bool initialized;
    
    void sendData(uint8_t r, uint8_t g, uint8_t b);
    void applyBrightness(uint8_t& r, uint8_t& g, uint8_t& b, uint8_t brightness);
};

#endif // WS2812B_H
