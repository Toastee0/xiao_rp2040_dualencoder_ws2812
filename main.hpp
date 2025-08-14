#pragma once

#include "pico/stdlib.h"
#include "encoder.hpp"
#include "ws2812_leds.hpp"

// ===================================================================
// ============== USER CONFIGURATION - SYSTEM SETUP ===============
// ===================================================================

// Number of encoders to use (1-4)
#define NUMBER_OF_ENCODERS 4

// Encoder interrupt GPIO pins - configure these for your hardware
#define ENCODER_1_IRQ_GPIO 1
#define ENCODER_2_IRQ_GPIO 2
#define ENCODER_3_IRQ_GPIO 3
#define ENCODER_4_IRQ_GPIO 4

// WS2812 LED strip configuration
#define WS2812_PIN 0
#define NUM_LEDS 117

// I2C configuration for encoders
#define I2C_SDA_GPIO 6
#define I2C_SCL_GPIO 7

// ===================================================================
// ================ MAIN APPLICATION API ===========================
// ===================================================================

// Initialize all systems (USB serial, encoders, LEDs, etc.)
bool init_systems();

// Main application loop - handles serial terminal and system coordination
void run_main_loop();

// Handle non-blocking serial terminal commands
void handle_serial_commands();

// System status
bool is_system_ready();

// Get ready encoder count
uint8_t get_ready_encoder_count();