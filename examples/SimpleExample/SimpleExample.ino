/*
 * ESP32_WS2812B Simple Example
 * 
 * Copyright (c) 2026 Xorlent
 * Licensed under the MIT License.
 * https://github.com/Xorlent/ESP32_WS2812B
 * 
 * This example demonstrates the minimal WS2812B library.
 * Connect your WS2812B LED data pin to GPIO 35 (default for M5 AtomS3 Lite) or change LED_PIN below.
 * 
 * The LED will cycle through all available colors.
 */

#include <WS2812B.h>

#define LED_PIN 35  // GPIO pin connected to WS2812B DIN

WS2812B led;

void setup() {
  Serial.begin(115200);
  
  // Initialize the LED on GPIO 35
  if (led.begin(LED_PIN)) {
    Serial.println("WS2812B initialized");
  } else {
    Serial.println("Failed to initialize WS2812B RGB LED");
    while(1); // Halt
  }
}

void loop() {
  // Cycle through colors with different brightness levels
  
  Serial.println("Red - Full brightness");
  led.set("red", 255);
  delay(1000);
  
  Serial.println("Green - Full brightness");
  led.set("green", 255);
  delay(1000);
  
  Serial.println("Blue - Full brightness");
  led.set("blue", 255);
  delay(1000);
  
  Serial.println("Purple - Full brightness");
  led.set("purple", 255);
  delay(1000);
  
  Serial.println("Yellow - Full brightness");
  led.set("yellow", 255);
  delay(1000);
  
  Serial.println("Orange - Full brightness");
  led.set("orange", 255);
  delay(1000);
  
  Serial.println("White - Full brightness");
  led.set("white", 255);
  delay(1000);
  
  Serial.println("White - 50% brightness");
  led.set("white", 128);
  delay(1000);
  
  Serial.println("White - 10% brightness");
  led.set("white", 25);
  delay(1000);
  
  Serial.println("Off (black)");
  led.set("black");
  delay(1000);
}
