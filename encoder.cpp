//important coding notes. 
// this is a realtime application, and sleep and printf calls are expensive and only allowed outside of irq functions.
// sleep is to be avoided outside the initial setup of the main loop, and in the case that the bootloader mode has been requested (because we are stopping anyway its ok to sleep now)
// it's important to ensure that the code does not "block" or have any delays, polling must be non-blocking in the same manner as our serial read function
// encoder module handles hardware initialization for encoders, but gets pin mappings and interrupt irq's from information added at the top of the main section as #defines
// encoder data will be exposed for future use by other code on the mcu

#include "encoder.hpp"
#include "main.hpp"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "libraries/breakout_encoder/breakout_encoder.hpp"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// ===================================================================
// ================ SYSTEM CONSTANTS AND GLOBALS ===================
// ===================================================================

// Use I2C configuration from main.hpp
#define I2C_PORT i2c1
#define I2C_BAUDRATE 400000  // 400kHz for responsive operation

// Encoder register addresses
#define ENCODER_COUNTER_REG 0x40    // Counter register
#define ENCODER_STATUS_REG 0x00     // Status register 
#define ENCODER_CONFIG_REG 0x00     // Configuration register
#define ENCODER_ADDRESS_REG 0x45    // I2C address register

// Expected I2C addresses for each encoder
static const uint8_t ENCODER_ADDRESSES[4] = {0x0F, 0x0E, 0x0D, 0x0C};

// IRQ GPIO pins from main.hpp configuration
static const uint8_t ENCODER_IRQ_PINS[4] = {
    ENCODER_1_IRQ_GPIO,
    ENCODER_2_IRQ_GPIO, 
    ENCODER_3_IRQ_GPIO,
    ENCODER_4_IRQ_GPIO
};

// Global encoder system state
static EncoderData encoders[4] = {0};
static uint8_t ready_encoder_count = 0;
static volatile bool irq_triggered[4] = {false};

// I2C and BreakoutEncoder instances - will be initialized in init_encoder_system()
static pimoroni::I2C* i2c_instance = nullptr;
static pimoroni::BreakoutEncoder* encoder_instances[4] = {nullptr};

// ===================================================================
// ================ HARDWARE INTERRUPT HANDLERS ====================
// ===================================================================

// GPIO interrupt callback function
void gpio_irq_callback(uint gpio, uint32_t events) {
    // Find which encoder triggered the interrupt
    for (int i = 0; i < NUMBER_OF_ENCODERS; i++) {
        if (gpio == ENCODER_IRQ_PINS[i]) {
            irq_triggered[i] = true;  // Set flag for main loop to process
            break;  // Found the pin, no need to continue searching
        }
    }
}

// ===================================================================
// ================ I2C COMMUNICATION FUNCTIONS ====================
// ===================================================================

// Read from encoder I2C register
static bool read_encoder_register(uint8_t encoder_addr, uint8_t reg, uint8_t* data, size_t len) {
    // Write register address
    int ret = i2c_write_blocking(I2C_PORT, encoder_addr, &reg, 1, true);
    if (ret < 0) return false;
    
    // Read data
    ret = i2c_read_blocking(I2C_PORT, encoder_addr, data, len, false);
    return ret >= 0;
}

// Write to encoder I2C register
static bool write_encoder_register(uint8_t encoder_addr, uint8_t reg, uint8_t* data, size_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    for (size_t i = 0; i < len; i++) {
        buffer[i + 1] = data[i];
    }
    
    int ret = i2c_write_blocking(I2C_PORT, encoder_addr, buffer, len + 1, false);
    return ret >= 0;
}

// Read 32-bit counter value from encoder
static bool read_encoder_counter(uint8_t encoder_idx) {
    if (encoder_idx >= NUMBER_OF_ENCODERS || !encoders[encoder_idx].ready) {
        return false;
    }
    
    uint8_t counter_bytes[4];
    if (!read_encoder_register(ENCODER_ADDRESSES[encoder_idx], ENCODER_COUNTER_REG, counter_bytes, 4)) {
        return false;
    }
    
    // Convert bytes to 32-bit signed integer (little-endian)
    int32_t new_counter = (int32_t)((counter_bytes[3] << 24) | 
                                   (counter_bytes[2] << 16) | 
                                   (counter_bytes[1] << 8) | 
                                   counter_bytes[0]);
    
    encoders[encoder_idx].previous_counter = encoders[encoder_idx].counter;
    encoders[encoder_idx].counter = new_counter;
    
    // Calculate delta (change since last read)
    encoders[encoder_idx].delta = new_counter - encoders[encoder_idx].previous_counter;
    
    return true;
}

// ===================================================================
// ================ ENCODER DETECTION AND SETUP ====================
// ===================================================================

// Test if encoder is present at given address
static bool test_encoder_presence(uint8_t encoder_addr) {
    uint8_t test_data;
    return read_encoder_register(encoder_addr, ENCODER_STATUS_REG, &test_data, 1);
}

// Initialize single encoder
static bool init_single_encoder(uint8_t encoder_idx) {
    if (encoder_idx >= NUMBER_OF_ENCODERS) {
        printf("ERROR: Invalid encoder index %d (max %d)\n", encoder_idx, NUMBER_OF_ENCODERS - 1);
        return false;
    }
    
    uint8_t encoder_addr = ENCODER_ADDRESSES[encoder_idx];
    uint8_t irq_pin = ENCODER_IRQ_PINS[encoder_idx];
    
    printf("Initializing encoder %d: Address=0x%02X, IRQ=GPIO%d\n", encoder_idx + 1, encoder_addr, irq_pin);
    
    // Test if encoder is present
    printf("Testing encoder presence at address 0x%02X...\n", encoder_addr);
    if (!test_encoder_presence(encoder_addr)) {
        printf("Encoder %d not found at address 0x%02X\n", encoder_idx + 1, encoder_addr);
        return false;
    }
    printf("✓ Encoder %d found at address 0x%02X\n", encoder_idx + 1, encoder_addr);
    
    // Initialize encoder data structure
    encoders[encoder_idx].encoder_id = encoder_idx;
    encoders[encoder_idx].i2c_address = encoder_addr;
    encoders[encoder_idx].irq_pin = irq_pin;
    encoders[encoder_idx].counter = 0;
    encoders[encoder_idx].previous_counter = 0;
    encoders[encoder_idx].delta = 0;
    encoders[encoder_idx].ready = true;  // Set ready BEFORE trying to read counter
    
    // Read initial counter value
    if (!read_encoder_counter(encoder_idx)) {
        printf("Failed to read initial counter from encoder %d\n", encoder_idx + 1);
        encoders[encoder_idx].ready = false;  // Reset ready flag on failure
        return false;
    }
    
    // Create BreakoutEncoder instance for RGB LED control
    if (i2c_instance == nullptr) {
        printf("Creating I2C instance on GPIO %d (SDA) and %d (SCL)\n", I2C_SDA_GPIO, I2C_SCL_GPIO);
        i2c_instance = new pimoroni::I2C(I2C_SDA_GPIO, I2C_SCL_GPIO);
    }
    
    printf("Creating BreakoutEncoder instance for encoder %d (Addr:0x%02X, IRQ:GPIO%d)\n", 
           encoder_idx + 1, encoder_addr, irq_pin);
    encoder_instances[encoder_idx] = new pimoroni::BreakoutEncoder(i2c_instance, encoder_addr, irq_pin);
    
    // Initialize the encoder
    printf("Initializing BreakoutEncoder instance for encoder %d...\n", encoder_idx + 1);
    if (!encoder_instances[encoder_idx]->init()) {
        printf("Failed to initialize BreakoutEncoder instance for encoder %d\n", encoder_idx + 1);
        delete encoder_instances[encoder_idx];
        encoder_instances[encoder_idx] = nullptr;
        return false;
    }
    printf("BreakoutEncoder instance for encoder %d initialized successfully\n", encoder_idx + 1);
    
    // Set initial RGB color based on encoder ID
    uint8_t colors[4][3] = {
        {255, 0, 0},   // Encoder 1: Red
        {0, 255, 0},   // Encoder 2: Green  
        {0, 0, 255},   // Encoder 3: Blue
        {255, 255, 0}  // Encoder 4: Yellow
    };
    
    printf("Setting RGB LED color for encoder %d...\n", encoder_idx + 1);
    encoder_instances[encoder_idx]->set_led(colors[encoder_idx][0], colors[encoder_idx][1], colors[encoder_idx][2]);
    
    encoders[encoder_idx].ready = true;
    ready_encoder_count++;
    
    printf("✓ Encoder %d initialized (Addr:0x%02X, IRQ:GPIO%d, RGB:%s)\n", 
           encoder_idx + 1, encoder_addr, irq_pin, 
           (encoder_idx == 0) ? "Red" : (encoder_idx == 1) ? "Green" : (encoder_idx == 2) ? "Blue" : "Yellow");
    
    return true;
}


// HSV to RGB conversion function
static void hsv_to_rgb(float h, float s, float v, uint8_t* r, uint8_t* g, uint8_t* b) {
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    
    float r_prime, g_prime, b_prime;
    
    if (h >= 0 && h < 60) {
        r_prime = c; g_prime = x; b_prime = 0;
    } else if (h >= 60 && h < 120) {
        r_prime = x; g_prime = c; b_prime = 0;
    } else if (h >= 120 && h < 180) {
        r_prime = 0; g_prime = c; b_prime = x;
    } else if (h >= 180 && h < 240) {
        r_prime = 0; g_prime = x; b_prime = c;
    } else if (h >= 240 && h < 300) {
        r_prime = x; g_prime = 0; b_prime = c;
    } else {
        r_prime = c; g_prime = 0; b_prime = x;
    }
    
    *r = (uint8_t)((r_prime + m) * 255);
    *g = (uint8_t)((g_prime + m) * 255);
    *b = (uint8_t)((b_prime + m) * 255);
}

// Update RGB LED color based on encoder position
static void update_encoder_rgb(uint8_t encoder_idx) {
    if (encoder_idx >= NUMBER_OF_ENCODERS || !encoders[encoder_idx].ready || encoder_instances[encoder_idx] == nullptr) {
        return;
    }
    
    // Constants for color calculation
    const int STEPS_PER_REV = 24;  // 24 steps per full rotation
    
    // Calculate normalized position (0.0 to 1.0)
    int32_t counter = encoders[encoder_idx].counter;
    while(counter < 0) counter += STEPS_PER_REV;  // Handle negative values
    
    float position = (float)(counter % STEPS_PER_REV) / (float)STEPS_PER_REV;
    
    // Each encoder gets a different base hue offset for identification
    float base_hue_offsets[4] = {0.0f, 90.0f, 180.0f, 270.0f};  // Red, Green, Blue, Magenta base colors
    float hue = fmodf(position * 360.0f + base_hue_offsets[encoder_idx], 360.0f);
    
    // Convert HSV to RGB (full saturation and brightness)
    uint8_t r, g, b;
    hsv_to_rgb(hue, 1.0f, 1.0f, &r, &g, &b);
    
    // Set the RGB LED color
    encoder_instances[encoder_idx]->set_led(r, g, b);
}

// ===================================================================
// ================ PUBLIC API IMPLEMENTATION ======================
// ===================================================================

bool init_encoder_system() {
    printf("=== Starting encoder system initialization ===\n");
    printf("NUMBER_OF_ENCODERS = %d\n", NUMBER_OF_ENCODERS);
    
    // Setup GPIO interrupts FIRST - assume encoders are present if configured
    printf("Setting up encoder GPIO interrupts...\n");
    bool first_interrupt = true;
    for (int i = 0; i < NUMBER_OF_ENCODERS; i++) {
        uint8_t irq_pin = ENCODER_IRQ_PINS[i];
        
        // Setup GPIO pin for interrupt
        gpio_init(irq_pin);
        gpio_set_dir(irq_pin, GPIO_IN);
        gpio_pull_up(irq_pin);  // Enable pull-up
        
        if (first_interrupt) {
            // First encoder: set callback and enable interrupt
            gpio_set_irq_enabled_with_callback(irq_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
            printf("✓ GPIO %d interrupt enabled with callback\n", irq_pin);
            first_interrupt = false;
        } else {
            // Subsequent encoders: just enable interrupt (callback already set)
            gpio_set_irq_enabled(irq_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
            printf("✓ GPIO %d interrupt enabled\n", irq_pin);
        }
    }
    
    // Initialize I2C bus after IRQ setup
    printf("Initializing I2C bus...\n");
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    
    printf("I2C initialized on GPIO%d(SDA) and GPIO%d(SCL) at %dkHz\n", 
           I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_BAUDRATE / 1000);
    
    // Initialize each configured encoder
    ready_encoder_count = 0;
    printf("Attempting to initialize %d encoders...\n", NUMBER_OF_ENCODERS);
    for (int i = 0; i < NUMBER_OF_ENCODERS; i++) {
        printf("\n--- Initializing encoder %d ---\n", i + 1);
        if (init_single_encoder(i)) {
            printf("✓ Encoder %d initialization successful\n", i + 1);
        } else {
            printf("✗ Encoder %d initialization failed\n", i + 1);
        }
    }
    
    if (ready_encoder_count > 0) {
        printf("Encoder system initialized with %d of %d encoders\n", 
               ready_encoder_count, NUMBER_OF_ENCODERS);
        return true;
    } else {
        printf("No encoders found - check wiring and addresses\n");
        return false;
    }
}

void update_encoder_system() {
    // Check each encoder for pending interrupts
    for (int i = 0; i < NUMBER_OF_ENCODERS; i++) {
        if (irq_triggered[i]) {
            // Print info about which interrupt was received
            printf("[IRQ%d-GPIO%d] Interrupt received for encoder %d ", 
                   i + 1, ENCODER_IRQ_PINS[i], i + 1);
            
            // Clear the interrupt flag (main loop clears the flag)
            irq_triggered[i] = false;
            
            // Read the encoder value using BreakoutEncoder library method
            int32_t new_counter = encoder_instances[i]->read();
            encoder_instances[i]->clear_interrupt_flag();
            
            if (new_counter != encoders[i].counter) {
                encoders[i].previous_counter = encoders[i].counter;
                encoders[i].counter = new_counter;
                encoders[i].delta = new_counter - encoders[i].previous_counter;
                
                printf("ENC%d CHANGED: %ld->%ld (delta:%ld) ", 
                       i + 1, encoders[i].previous_counter, encoders[i].counter, encoders[i].delta);
                update_encoder_rgb(i);     // Update RGB LED based on position
            } else {
                printf("ENC%d no change (still %ld) ", i + 1, encoders[i].counter);
            }
        }
    }
}

const EncoderData* get_encoder_data(uint8_t encoder_id) {
    if (encoder_id >= NUMBER_OF_ENCODERS || !encoders[encoder_id].ready) {
        return NULL;
    }
    return &encoders[encoder_id];
}

uint8_t get_encoder_ready_count() {
    return ready_encoder_count;
}

bool is_encoder_ready(uint8_t encoder_id) {
    if (encoder_id >= NUMBER_OF_ENCODERS) {
        return false;
    }
    return encoders[encoder_id].ready;
}

int32_t get_encoder_counter(uint8_t encoder_id) {
    if (encoder_id >= NUMBER_OF_ENCODERS || !encoders[encoder_id].ready) {
        return 0;
    }
    return encoders[encoder_id].counter;
}

int32_t get_encoder_delta(uint8_t encoder_id) {
    if (encoder_id >= NUMBER_OF_ENCODERS || !encoders[encoder_id].ready) {
        return 0;
    }
    return encoders[encoder_id].delta;
}

void change_encoder_address(uint8_t encoder_id) {
    if (encoder_id >= NUMBER_OF_ENCODERS) {
        printf("Invalid encoder ID: %d\n", encoder_id);
        return;
    }
    
    uint8_t new_address = ENCODER_ADDRESSES[encoder_id];
    printf("Setting encoder %d to address 0x%02X...\n", encoder_id + 1, new_address);
    
    // Find any device on I2C bus
    uint8_t found_address = 0;
    bool found = false;
    
    for (int addr = 0x08; addr < 0x78; addr++) {
        if (test_encoder_presence(addr)) {
            found_address = addr;
            found = true;
            break;
        }
    }
    
    if (!found) {
        printf("No I2C device found\n");
        return;
    }
    
    printf("Found device at 0x%02X, changing to 0x%02X\n", found_address, new_address);
    
    // Create temporary encoder instance for address change
    pimoroni::BreakoutEncoder temp_encoder(i2c_instance, found_address);
    
    // Set the new address
    temp_encoder.set_address(new_address);
    
    printf("Address change complete\n");
}

void print_encoder_status() {
    printf("\n=== Encoder System Status ===\n");
    printf("Ready encoders: %d of %d\n", ready_encoder_count, NUMBER_OF_ENCODERS);
    
    for (int i = 0; i < NUMBER_OF_ENCODERS; i++) {
        if (encoders[i].ready) {
            printf("Encoder %d: Counter=%ld, Delta=%ld, Addr=0x%02X, IRQ=GPIO%d\n",
                   i + 1, 
                   encoders[i].counter,
                   encoders[i].delta,
                   encoders[i].i2c_address,
                   encoders[i].irq_pin);
        } else {
            printf("Encoder %d: Not ready (expected Addr=0x%02X, IRQ=GPIO%d)\n",
                   i + 1,
                   ENCODER_ADDRESSES[i],
                   ENCODER_IRQ_PINS[i]);
        }
    }
    printf("\n");
}

// Scan I2C bus for devices
void scan_i2c_bus() {
    printf("Scanning I2C bus (GPIO%d=SDA, GPIO%d=SCL)...\n", I2C_SDA_GPIO, I2C_SCL_GPIO);
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    bool found_any = false;
    
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x: ", addr);
        }
        
        // Reserved addresses
        int ret;
        uint8_t rxdata;
        if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78) {
            printf("   ");
        } else {
            ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
            if (ret >= 0) {
                printf("%02x ", addr);
                found_any = true;
            } else {
                printf("-- ");
            }
        }
        
        if (addr % 16 == 15) {
            printf("\n");
        }
    }
    
    if (found_any) {
        printf("\nFound devices! Expected encoder addresses:\n");
        printf("  Encoder 1: 0x0F, Encoder 2: 0x0E, Encoder 3: 0x0D, Encoder 4: 0x0C\n");
        printf("Use address change commands (a1, a2, a3, a4) to configure encoders\n");
    } else {
        printf("\nNo I2C devices found. Check wiring and power.\n");
    }
    printf("\n");
}
