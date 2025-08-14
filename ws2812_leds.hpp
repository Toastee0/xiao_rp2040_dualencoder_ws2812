#pragma once

#include "pico/stdlib.h"
#include <cstdint>

// ===================================================================
// ================ WS2812 LED CONFIGURATION =======================
// ===================================================================

// WS2812_PIN and NUM_LEDS are defined in main.hpp
// This allows the main configuration to control all pin assignments

#ifndef BALL_SIZE
#define BALL_SIZE 7                     // Size of the light ball in pixels
#endif

// ===================================================================
// ================ LED ANIMATION TYPES ============================
// ===================================================================

enum AnimationMode {
    LED_OFF,                    // All LEDs off
    RAINBOW_CYCLE,             // Rainbow cycle animation
    COLOR_WIPE,                // Sequential LED color filling
    THEATER_CHASE,             // Theater chase effect
    SPARKLE,                   // Random twinkling effects
    PULSE,                     // Breathing color effects
    ENCODER_CONTROL            // Interactive encoder control mode
};

// ===================================================================
// ================ PIXEL ART OBJECT SYSTEM ========================
// ===================================================================

#define MAX_PIXEL_OBJECTS 16   // Maximum number of stored pixel art objects

struct PixelObject {
    bool active;               // Is this object slot in use?
    uint16_t position;         // Center LED position (0-106)
    uint8_t size;             // Object size in pixels (1-20)
    uint8_t red;              // RGB color values
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;       // Overall brightness (0-255)
};

// ===================================================================
// ================ ENCODER CONTROL STATE ==========================
// ===================================================================

struct EncoderControlState {
    uint16_t current_position; // Current LED position (Encoder 1)
    uint16_t current_hue;      // Current color hue (Encoder 2) 
    uint8_t current_size;      // Current object size (Encoder 3)
    int32_t last_encoder4_value; // Last Encoder 4 value for delta detection
};

// ===================================================================
// ================ PUBLIC API FUNCTIONS ===========================
// ===================================================================

// Initialize WS2812 LED system
bool init_ws2812_system();

// Update WS2812 system (run animations)
void update_ws2812_system();

// Animation mode control
void next_animation_mode();
void set_animation_mode(AnimationMode mode);
AnimationMode get_current_animation_mode();
const char* get_animation_mode_name();

// Pixel art object management
void save_current_pixel_object();
void delete_pixel_object_at_position(uint16_t position);
void clear_all_pixel_objects();
uint8_t get_active_object_count();

// Color utility functions
uint32_t rgb_to_grb(uint8_t r, uint8_t g, uint8_t b);
uint32_t hsv_to_grb(uint16_t h, uint8_t s, uint8_t v);
uint32_t blend_colors(uint32_t color1, uint32_t color2, uint8_t alpha);
uint32_t dim_color(uint32_t color, uint8_t brightness);
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
