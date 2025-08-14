//important coding notes. 
// this is a realtime application, and sleep and printf calls are expensive and only allowed outside of irq functions.
// sleep is to be avoided outside the initial setup of the main loop, and in the case that the bootloader mode has been requested (because we are stopping anyway its ok to sleep now)
// it's important to ensure that the code does not "block" or have any delays, polling must be non-blocking in the same manner as our serial read function
// the goal is to create a basic framework for managing and using multiple encoders with clean data provided to any code that needs it.
// as an example this project uses ws2812 as an example of how to use the encoder data in a realtime application
// the leds should be controlled in an easily removed module so other people can adapt this easily without a led strip to test with.

#include "main.hpp"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include <stdio.h>

// ===================================================================
// ================ DUAL CORE FUNCTIONS ============================
// ===================================================================

// Core 1 entry point - dedicated to LED animations
void core1_entry() {
    printf("Core 1 started - handling LED animations\n");
    
    while (true) {
        // Update WS2812 LED animations on Core 1
        update_ws2812_system();
        
        // No sleep/delay - let the LED update function control timing
        // This allows for smooth animations while Core 0 handles interrupts
    }
}

// ===================================================================
// ================ SYSTEM GLOBALS ==================================
// ===================================================================

static bool system_ready = false;
static bool toggle = false;

// ===================================================================
// ================ SERIAL TERMINAL FUNCTIONS ======================
// ===================================================================

// Non-blocking serial read function
static char read_serial_nonblocking() {
    int c = getchar_timeout_us(0);  // 0 timeout = non-blocking
    if (c == PICO_ERROR_TIMEOUT) {
        return 0;  // No character available
    }
    return (char)c;  // Character received
}

// Handle address change commands
static void handle_address_change(char encoder_num) {
    uint8_t encoder_idx = encoder_num - '1';  // Convert '1'-'4' to 0-3
    
    if (encoder_idx >= NUMBER_OF_ENCODERS) {
        printf("Encoder %c not configured (only %d encoders enabled)\n", encoder_num, NUMBER_OF_ENCODERS);
        return;
    }
    
    // Call encoder module's address change function
    change_encoder_address(encoder_idx);
}

// Simple I2C scanner function
static void scan_i2c() {
    printf("\nScanning I2C bus:\n");
    scan_i2c_bus();  // Call the actual implementation in encoder module
}

// ===================================================================
// ================ PUBLIC API IMPLEMENTATION ======================
// ===================================================================

bool init_systems() {
    // Initialize onboard LED for status indication
#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

    // Initialize USB serial terminal
    stdio_init_all();
    
    printf("=== XIAO RP2040 %d-Encoder LED Animation System ===\n", NUMBER_OF_ENCODERS);
    printf("I2C Pins: SDA=%d, SCL=%d\n", I2C_SDA_GPIO, I2C_SCL_GPIO);
    
    // Initialize WS2812 LED system
    if (!init_ws2812_system()) {
        printf("✗ Failed to initialize WS2812 LED system\n");
        return false;
    }
    printf("✓ WS2812 LED system initialized (%d LEDs on GPIO %d)\n", NUM_LEDS, WS2812_PIN);
    
    // Initialize encoder system
    if (!init_encoder_system()) {
        printf("✗ Failed to initialize encoder system\n");
        return false;
    }
    
    uint8_t ready_encoders = get_ready_encoder_count();
    if (ready_encoders > 0) {
        printf("✓ %d of %d encoders ready!\n", ready_encoders, NUMBER_OF_ENCODERS);
        
        // Launch Core 1 for LED animations
        printf("Launching Core 1 for LED animation processing...\n");
        multicore_launch_core1(core1_entry);
        
        system_ready = true;
    } else {
        printf("✗ No encoders found!\n");
        printf("Check:\n");
        printf("  - Wiring: SDA->GPIO%d, SCL->GPIO%d\n", I2C_SDA_GPIO, I2C_SCL_GPIO);
        printf("  - Power: VCC and GND connected\n");
        printf("Expected encoder configurations:\n");
        printf("  - Encoder 1: INT->GPIO%d, Address=0x0F\n", ENCODER_1_IRQ_GPIO);
        printf("  - Encoder 2: INT->GPIO%d, Address=0x0E\n", ENCODER_2_IRQ_GPIO);
        printf("  - Encoder 3: INT->GPIO%d, Address=0x0D\n", ENCODER_3_IRQ_GPIO);
        printf("  - Encoder 4: INT->GPIO%d, Address=0x0C\n", ENCODER_4_IRQ_GPIO);
        printf("Use 's' command to scan I2C bus\n");
        
#ifdef PICO_DEFAULT_LED_PIN
        printf("Setting onboard LED on to indicate error\n");
        gpio_put(PICO_DEFAULT_LED_PIN, true);
#endif
        return false;
    }
    
    printf("Starting main loop...\n");
    return true;
}

// Soft reset - re-initialize systems without CPU reset
static void soft_reset_system() {
    printf("=== Soft Reset Starting ===\n");
    
    // Reset system state
    system_ready = false;
    
    // Re-initialize all systems (skip stdio_init_all since serial is already working)
    printf("Re-initializing WS2812 LED system...\n");
    if (!init_ws2812_system()) {
        printf("✗ Failed to re-initialize WS2812 LED system\n");
    } else {
        printf("✓ WS2812 LED system re-initialized\n");
    }
    
    printf("Re-initializing encoder system...\n");
    if (!init_encoder_system()) {
        printf("✗ Failed to re-initialize encoder system\n");
    } else {
        uint8_t ready_encoders = get_ready_encoder_count();
        printf("✓ Encoder system re-initialized with %d encoders\n", ready_encoders);
        system_ready = true;
    }
    
    printf("=== Soft Reset Complete ===\n");
}

// Print debug information
static void print_debug_info() {
    printf("=== System Debug Information ===\n");
    printf("System Ready: %s\n", system_ready ? "YES" : "NO");
    printf("Ready Encoders: %d of %d\n", get_ready_encoder_count(), NUMBER_OF_ENCODERS);
    
    // Print encoder details
    print_encoder_status();
    
    // Print WS2812 info
    printf("WS2812 LEDs: %d on GPIO %d\n", NUM_LEDS, WS2812_PIN);
    printf("I2C Bus: GPIO %d (SDA), GPIO %d (SCL)\n", I2C_SDA_GPIO, I2C_SCL_GPIO);
    
    printf("Expected Encoder Configuration:\n");
    printf("  Encoder 1: Addr=0x0F, IRQ=GPIO%d\n", ENCODER_1_IRQ_GPIO);
    printf("  Encoder 2: Addr=0x0E, IRQ=GPIO%d\n", ENCODER_2_IRQ_GPIO);
    printf("  Encoder 3: Addr=0x0D, IRQ=GPIO%d\n", ENCODER_3_IRQ_GPIO);
    printf("  Encoder 4: Addr=0x0C, IRQ=GPIO%d\n", ENCODER_4_IRQ_GPIO);
    printf("================================\n");
}

void handle_serial_commands() {
    char c = read_serial_nonblocking();
    if (c == 0) return;  // No character available
    
    if (c == 'a') {
        printf("\nEncoder Address Commands: a1, a2, a3, a4\n");
    } else if (c >= '1' && c <= '4') {
        handle_address_change(c);
    } else if (c == 's') {
        printf("\nScanning I2C bus:\n");
        scan_i2c();
    } else if (c == 'h') {
        printf("\n%d-Encoder System Commands:\n", NUMBER_OF_ENCODERS);
        printf("  'a1' - Set encoder 1 to address 0x0F\n");
        printf("  'a2' - Set encoder 2 to address 0x0E\n");
        printf("  'a3' - Set encoder 3 to address 0x0D\n");
        printf("  'a4' - Set encoder 4 to address 0x0C\n");
        printf("  'n' - Next animation mode\n");
        printf("  's' - Scan I2C bus\n");
        printf("  'r' - Soft reset (re-initialize systems)\n");
        printf("  'd' - Debug info (system status)\n");
        printf("  'z' - Reset to bootloader mode\n");
        printf("  'h' - Show this help\n");
    } else if (c == 'n') {
        next_animation_mode();
        printf("Animation mode: %s\n", get_animation_mode_name());
    } else if (c == 'z') {
        printf("Resetting to bootloader mode (BOOTSEL)...\n");
        sleep_ms(100);  // Give time for message to be sent (bootloader exception)
        reset_usb_boot(0, 0);
    } else if (c == 'r') {
        printf("Performing soft reset...\n");
        soft_reset_system();
    } else if (c == 'd') {
        print_debug_info();
    }
}

void run_main_loop() {
    if (!system_ready) {
        // If no encoders ready, just handle basic commands
        handle_serial_commands();
        return;
    }
    
    // Core 0: Handle encoder interrupts and serial commands
    // Core 1: Handle LED animations (runs in parallel)
    
    // Toggle onboard LED for status indication
#ifdef PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, toggle);
#endif
    toggle = !toggle;
    
    // Handle serial terminal commands (non-blocking)
    handle_serial_commands();
    
    // Process encoder interrupts and update system
    update_encoder_system();
    
    // No delays - pure interrupt-driven realtime operation
    // LED animations are handled on Core 1 in parallel
}

bool is_system_ready() {
    return system_ready;
}

uint8_t get_ready_encoder_count() {
    return get_encoder_ready_count();  // Delegate to encoder module
}

// ===================================================================
// ================ MAIN ENTRY POINT ===============================
// ===================================================================

int main() {
    // Initialize all systems
    if (!init_systems()) {
        // System initialization failed - stay in error loop
        while (true) {
            handle_serial_commands();  // Still allow basic commands
            sleep_ms(100);  // Error state allows sleep
        }
    }
    
    // Main realtime loop
    while (true) {
        run_main_loop();
        // No delay needed with hardware interrupts - they're instant!
        // Pure interrupt-driven - no delays!
    }
    
    return 0;
}
