#pragma once

#include "pico/stdlib.h"
#include <cstdint>

// ===================================================================
// ================ WS2812 LED CONFIGURATION =======================
// ===================================================================

#ifndef WS2812_PIN
#define WS2812_PIN 1                    // GPIO pin for WS2812 data line
#endif

#ifndef NUM_LEDS
#define NUM_LEDS 117                    // Number of LEDs in the strip
#endif

#ifndef BALL_SIZE
#define BALL_SIZE 7                     // Size of the light ball in pixels
#endif

// ===================================================================
// ================ LED ANIMATION TYPES ============================
// ===================================================================

enum AnimationMode {
    LED_OFF,                    // All LEDs off
    RAINBOW_CYCLE,             // Rainbow cycle animation
    ENCODER_POSITION,          // Show encoder positions
    ENCODER_DELTA,             // Show encoder speed/delta
    THEATER_CHASE,             // Theater chase effect
    SOLID_COLOR,               // Solid cycling color
    FIRE_EFFECT                // Fire simulation effect
};

// ===================================================================
// ================ PUBLIC API FUNCTIONS ===========================
// ===================================================================

// Initialize WS2812 LED system
bool init_ws2812_system();

// Update WS2812 system (run animations)
void update_ws2812_system();

// Color utility functions
uint32_t rgb_to_grb(uint8_t r, uint8_t g, uint8_t b);
uint32_t hsv_to_grb(uint16_t h, uint8_t s, uint8_t v);
uint32_t blend_colors(uint32_t color1, uint32_t color2, uint8_t alpha);
uint32_t dim_color(uint32_t color, uint8_t brightness);

// LED control functions
void clear_leds();
void set_led_color(uint16_t led_index, uint32_t color);
void set_led_range(uint16_t start, uint16_t end, uint32_t color);
void update_leds();

// Animation control
void set_animation_mode(AnimationMode mode);
AnimationMode get_animation_mode();
void cycle_animation_mode();
void set_all_leds_color(uint32_t color);
void set_led_brightness(uint8_t brightness);

// System status
bool is_ws2812_ready();

// Encoder-specific animations
void set_encoder_led_visualization();
void set_encoder_speed_visualization();

// Testing function
void test_led_pattern();
