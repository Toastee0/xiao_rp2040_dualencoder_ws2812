#pragma once

#include "pico/stdlib.h"
#include <cstdint>

// ===================================================================
// ============== USER CONFIGURATION - IRQ GPIOS ===================
// ===================================================================
// Configure the GPIO pins where you have connected your encoder interrupt lines.
// You can use any available GPIO pins on your XIAO RP2040.
// Set NUMBER_OF_ENCODERS in main.hpp to how many encoders you want to use (1-4).

// Default GPIO pins for encoder interrupts (user can override)
#ifndef ENCODER_IRQ_GPIOS
static const uint8_t ENCODER_IRQ_GPIOS[4] = {
    1,    // Encoder 1 interrupt GPIO (connect encoder 1 INT pin here)
    2,    // Encoder 2 interrupt GPIO (connect encoder 2 INT pin here) 
    3,     // Encoder 3 interrupt GPIO (connect encoder 3 INT pin here)
    4     // Encoder 4 interrupt GPIO (connect encoder 4 INT pin here)
};
#endif

// ===================================================================
// ================ ENCODER DATA STRUCTURE =========================
// ===================================================================

struct EncoderData {
    uint8_t encoder_id;               // Encoder ID (0-3)
    uint8_t i2c_address;              // I2C address of this encoder
    uint8_t irq_pin;                  // GPIO pin for interrupt
    int32_t counter;                  // Current encoder counter value
    int32_t previous_counter;         // Previous counter value for delta calculation
    int32_t delta;                    // Change since last read
    bool ready;                       // Encoder is initialized and working
    uint32_t last_update_time;        // Timestamp of last change
};

// ===================================================================
// ================ PUBLIC API FUNCTIONS ===========================
// ===================================================================

// Initialize the encoder system
bool init_encoder_system();

// Update encoder positions and colors
void update_encoder_system();

// Get encoder data for external use by encoder ID
const EncoderData* get_encoder_data(uint8_t encoder_id);

// Get number of ready encoders
uint8_t get_encoder_ready_count();

// Check if specific encoder is ready
bool is_encoder_ready(uint8_t encoder_id);

// Get encoder counter value
int32_t get_encoder_counter(uint8_t encoder_id);

// Get encoder delta (change) value
int32_t get_encoder_delta(uint8_t encoder_id);

// Set encoder I2C address
void change_encoder_address(uint8_t encoder_id);

// Print encoder status for debugging
void print_encoder_status();

// Scan I2C bus for devices
void scan_i2c_bus();

// Encoder system constants
extern const uint8_t ENCODER_I2C_ADDRESSES[4];
extern const uint8_t STEPS_PER_REV;