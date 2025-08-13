#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include <algorithm>  // For std::max
#include <cstdlib>    // For strtol
#include "pico/bootrom.h"
#include "pimoroni_common.hpp"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "ws2812.pio.h"

#include "breakout_encoder.hpp"

using namespace pimoroni;

// Pin configuration for XIAO RP2040
static const uint8_t ENCODER1_INTERRUPT_PIN = 26;  // First encoder interrupt pin
static const uint8_t ENCODER2_INTERRUPT_PIN = 27;  // Second encoder interrupt pin
static const uint8_t WS2812_PIN = 1;               // WS2812 strip pin
static const uint8_t STEPS_PER_REV = 24;
static const uint16_t NUM_LEDS = 117;              // Number of LEDs in the strip
static const uint8_t BALL_SIZE = 7;                // Size of the light ball in pixels (increased for wider ball)

// WS2812 control variables
PIO pio = pio0;
uint sm = 0;

// LED strip buffer
uint32_t led_buffer[NUM_LEDS];

// Trail persistence effect
float trail_brightness[NUM_LEDS];
static const float TRAIL_FADE_RATE = 0.92f;  // How fast trails fade (0.92 = 8% fade per frame)

// Encoder tracking
int16_t count1 = 0, count2 = 0;

// Interrupt flags
volatile bool enc1_interrupt = false;
volatile bool enc2_interrupt = false;

// Setup GPIO interrupts for encoders (handler defined after encoder objects)
void setup_encoder_interrupts();

I2C i2c(6,7);  // SDA=6, SCL=7 - default I2C pins for XIAO RP2040
BreakoutEncoder enc1(&i2c, BreakoutEncoder::DEFAULT_I2C_ADDRESS, ENCODER1_INTERRUPT_PIN);  // First encoder at 0x0F
BreakoutEncoder enc2(&i2c, 0x0E, ENCODER2_INTERRUPT_PIN);  // Second encoder at 0x0E (changed address)
bool toggle = false;

// Single GPIO interrupt handler that routes to the correct encoder
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == ENCODER1_INTERRUPT_PIN) {
        enc1_interrupt = true;
    } 
    else if (gpio == ENCODER2_INTERRUPT_PIN) {
        enc2_interrupt = true;
    }
}

// Now implement the setup function
void setup_encoder_interrupts() {
    // Set the global GPIO callback first, then enable both pins
    gpio_set_irq_enabled_with_callback(ENCODER1_INTERRUPT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    // Now enable the second pin (callback is already set globally)
    gpio_set_irq_enabled(ENCODER2_INTERRUPT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
    printf("✓ Both GPIO %d and %d interrupts enabled with shared callback\n", ENCODER1_INTERRUPT_PIN, ENCODER2_INTERRUPT_PIN);
}

// Debug function to change encoder I2C address
void change_encoder_address(uint8_t encoder_num, uint8_t new_address) {
    BreakoutEncoder* encoder = (encoder_num == 1) ? &enc1 : &enc2;
    uint8_t current_addr = (encoder_num == 1) ? BreakoutEncoder::DEFAULT_I2C_ADDRESS : 0x0E;
    
    printf("Changing encoder %d address from 0x%02X to 0x%02X\n", 
           encoder_num, current_addr, new_address);
    
    encoder->set_address(new_address);
    printf("Address change command sent to encoder %d (0x%02X)\n", encoder_num, new_address);
    printf("Note: This change is persistent in the encoder's flash memory.\n");
    printf("You'll need to update your code to use the new address.\n");
}

// Non-blocking serial read function
char read_serial_nonblocking() {
    int c = getchar_timeout_us(0);  // 0 timeout = non-blocking
    if (c == PICO_ERROR_TIMEOUT) {
        return 0;  // No character available
    }
    return (char)c;  // Character received
}

// Convert RGB values to 32-bit color value for WS2812
uint32_t rgb_to_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t) r << 8) | ((uint32_t) g << 16) | (uint32_t) b;
}

// Initialize WS2812 PIO
void ws2812_init() {
    uint offset = pio_add_program(pio, &ws2812_program);
    
    pio_gpio_init(pio, WS2812_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, WS2812_PIN, 1, true);
    
    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, WS2812_PIN);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    
    int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
    float div = clock_get_hz(clk_sys) / (800000 * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);
    
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// Send the LED buffer to the strip
void ws2812_show() {
    for (int i = 0; i < NUM_LEDS; i++) {
        pio_sm_put_blocking(pio, sm, led_buffer[i] << 8u);
    }
}

// Clear all LEDs
void clear_leds() {
    for (int i = 0; i < NUM_LEDS; i++) {
        led_buffer[i] = 0;
    }
}

// Set a single LED color
void set_led(uint16_t index, uint32_t color) {
    if (index < NUM_LEDS) {
        led_buffer[index] = color;
    }
}

// Create dual balls of light at specific positions with persistence trail
void draw_dual_balls(float position1, float position2) {
    // First, fade all trail brightness values
    for (int i = 0; i < NUM_LEDS; i++) {
        trail_brightness[i] *= TRAIL_FADE_RATE;
        if (trail_brightness[i] < 0.01f) trail_brightness[i] = 0.0f;
    }
    
    // Calculate the center positions
    int center1 = (int)round(position1);
    int center2 = (int)round(position2);
    
    // Draw first ball (cyan/blue) with brightness falloff and update trail
    for (int i = 0; i < BALL_SIZE; i++) {
        int led_pos = center1 - (BALL_SIZE / 2) + i;
        
        if (led_pos >= 0 && led_pos < NUM_LEDS) {
            float distance = abs(i - (BALL_SIZE / 2));
            float brightness = 1.0f - (distance / (float)(BALL_SIZE / 2));
            if (brightness < 0) brightness = 0;
            
            // Update trail brightness (keep max of current ball or existing trail)
            trail_brightness[led_pos] = std::max(brightness, trail_brightness[led_pos]);
        }
    }
    
    // Draw second ball (magenta/red) with brightness falloff and update trail
    for (int i = 0; i < BALL_SIZE; i++) {
        int led_pos = center2 - (BALL_SIZE / 2) + i;
        
        if (led_pos >= 0 && led_pos < NUM_LEDS) {
            float distance = abs(i - (BALL_SIZE / 2));
            float brightness = 1.0f - (distance / (float)(BALL_SIZE / 2));
            if (brightness < 0) brightness = 0;
            
            // For second ball, use a different approach - blend with existing
            trail_brightness[led_pos] = std::max(brightness * 0.8f, trail_brightness[led_pos]);
        }
    }
    
    // Set all LEDs based on trail brightness with position-based coloring
    for (int i = 0; i < NUM_LEDS; i++) {
        if (trail_brightness[i] > 0.01f) {
            // Distance to each ball center determines color mixing
            float dist1 = abs(i - center1);
            float dist2 = abs(i - center2);
            
            uint8_t r, g, b;
            if (dist1 < dist2) {
                // Closer to first ball - cyan/blue
                r = (uint8_t)(0 * trail_brightness[i]);
                g = (uint8_t)(255 * trail_brightness[i]);
                b = (uint8_t)(255 * trail_brightness[i]);
            } else {
                // Closer to second ball - magenta/red
                r = (uint8_t)(255 * trail_brightness[i]);
                g = (uint8_t)(0 * trail_brightness[i]);
                b = (uint8_t)(128 * trail_brightness[i]);
            }
            
            set_led(i, rgb_to_u32(r, g, b));
        } else {
            set_led(i, rgb_to_u32(0, 0, 0));
        }
    }
    
    ws2812_show();
}

// HSV Conversion expects float inputs in the range of 0.00-1.00 for each channel
// Outputs are rgb in the range 0-255 for each channel
void from_hsv(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
	float i = floor(h * 6.0f);
	float f = h * 6.0f - i;
	v *= 255.0f;
	uint8_t p = v * (1.0f - s);
	uint8_t q = v * (1.0f - f * s);
	uint8_t t = v * (1.0f - (1.0f - f) * s);

	switch (int(i) % 6) {
		case 0: r = v; g = t; b = p; break;
		case 1: r = q; g = v; b = p; break;
		case 2: r = p; g = v; b = t; break;
		case 3: r = p; g = q; b = v; break;
		case 4: r = t; g = p; b = v; break;
		case 5: r = v; g = p; b = q; break;
	}
}

void dual_encoder_update() {
    // Map encoder counts to LED strip positions (0 to NUM_LEDS-1)
    float position1 = ((float)count1 / (float)STEPS_PER_REV) * (NUM_LEDS - 1);
    float position2 = ((float)count2 / (float)STEPS_PER_REV) * (NUM_LEDS - 1);
    
    // Keep positions within bounds
    while (position1 < 0) position1 += NUM_LEDS;
    while (position1 >= NUM_LEDS) position1 -= NUM_LEDS;
    while (position2 < 0) position2 += NUM_LEDS;
    while (position2 >= NUM_LEDS) position2 -= NUM_LEDS;
    
    // Calculate colors for encoder LEDs based on positions
    float h1 = position1 / (float)NUM_LEDS;
    float h2 = position2 / (float)NUM_LEDS;
    uint8_t r1 = 0, g1 = 0, b1 = 0;
    uint8_t r2 = 0, g2 = 0, b2 = 0;
    from_hsv(h1, 1.0f, 1.0f, r1, g1, b1);
    from_hsv(h2, 1.0f, 1.0f, r2, g2, b2);
    
    // Update encoder LED colors
    enc1.set_led(r1, g1, b1);
    enc2.set_led(r2, g2, b2);
    
    // Draw dual balls on the LED strip
    draw_dual_balls(position1, position2);
    
    printf("Enc1: %d->%.1f, Enc2: %d->%.1f\n", count1, position1, count2, position2);
}// Simple I2C scanner function
void scan_i2c() {
	printf("Scanning I2C bus...\n");
	bool found_any = false;
	
	for (int addr = 1; addr < 128; addr++) {
		uint8_t data;
		int ret = i2c.read_blocking(addr, &data, 1, false);
		if (ret >= 0) {
			printf("  Found device at address 0x%02X\n", addr);
			found_any = true;
		}
	}
	
	if (!found_any) {
		printf("  No I2C devices found!\n");
	}
}

int main() {
#ifdef PICO_DEFAULT_LED_PIN
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

	stdio_init_all();
	
	printf("LED Encoder starting...\n");
	printf("Initializing WS2812 LED strip (%d LEDs on GPIO %d)\n", NUM_LEDS, WS2812_PIN);
	
	// Initialize WS2812 LED strip
	ws2812_init();
	clear_leds();
	ws2812_show();
	
	printf("Initializing encoder...\n");
	
	
	
	printf("=== XIAO RP2040 Dual Encoder Demo Starting ===\n");
	printf("I2C Pins: SDA=%d, SCL=%d\n", 6, 7);
	printf("Encoder 1 Interrupt Pin: %d (Address: 0x%02X)\n", ENCODER1_INTERRUPT_PIN, BreakoutEncoder::DEFAULT_I2C_ADDRESS);
	printf("Encoder 2 Interrupt Pin: %d (Address: 0x0E)\n", ENCODER2_INTERRUPT_PIN);
	printf("Looking for encoders...\n");

	bool enc1_ready = false, enc2_ready = false;
	
	if(enc1.init()) {
		printf("✓ Encoder 1 found and initialized!\n");
		enc1_ready = true;
		//enc1.set_direction(BreakoutEncoder::DIRECTION_CCW);    // Uncomment this to flip the direction
	} else {
		printf("✗ Encoder 1 not found at address 0x%02X\n", BreakoutEncoder::DEFAULT_I2C_ADDRESS);
	}
	
	if(enc2.init()) {
		printf("✓ Encoder 2 found and initialized!\n");
		enc2_ready = true;
		//enc2.set_direction(BreakoutEncoder::DIRECTION_CCW);    // Uncomment this to flip the direction
	} else {
		printf("✗ Encoder 2 not found at address 0x0E\n");
		printf("  Use 'a' command to change an encoder to address 0x0E\n");
	}
	
	if (enc1_ready || enc2_ready) {
		// Setup hardware interrupts
		setup_encoder_interrupts();
		
		// Initialize display
		dual_encoder_update();
		if (enc1_ready) enc1.clear_interrupt_flag();
		if (enc2_ready) enc2.clear_interrupt_flag();
		printf("Starting main loop...\n");

		while(true) {
#ifdef PICO_DEFAULT_LED_PIN
			gpio_put(PICO_DEFAULT_LED_PIN, toggle);
#endif
			toggle = !toggle;

			// Check for serial commands
			int c = getchar_timeout_us(0);  // Non-blocking getchar
			if (c != PICO_ERROR_TIMEOUT) {
				if (c == 'a') {
					printf("\nPress '1' for encoder 1 or '2' for encoder 2 address change\n");
				} else if (c == '1') {
					printf("Changing encoder 1 to address 0x0E...\n");
					change_encoder_address(1, 0x0E);
				} else if (c == '2') {
					printf("Changing encoder 2 to address 0x0F...\n");  
					change_encoder_address(2, BreakoutEncoder::DEFAULT_I2C_ADDRESS);
					printf("Commands: 'a' = change address, 's' = scan I2C\n");
				} else if (c == 's') {
					printf("\nScanning I2C bus:\n");
					scan_i2c();
				} else if (c == 'h') {
					printf("\nDual Encoder Commands:\n");
					printf("  'a' - Address change menu\n");
					printf("  '1' - Change encoder 1 to address 0x0E\n");
					printf("  '2' - Change encoder 2 to address 0x0F\n");
					printf("  's' - Scan I2C bus\n");
					printf("  'z' - Reset to bootloader mode\n");
					printf("  'h' - Show this help\n");
				} else if (c == 'z') {
					printf("Resetting to bootloader mode (BOOTSEL)...\n");
					sleep_ms(100);  // Give time for message to be sent
					reset_usb_boot(0, 0);
				}
			}

			// Check if any encoder interrupted (do I2C work in main loop)
			if(enc1_interrupt) {
				enc1_interrupt = false;  // Clear our flag immediately
				printf("[IRQ1-GPIO26] ");  // Debug info in main loop
				
				int16_t new_count1 = enc1.read();
				enc1.clear_interrupt_flag();  // Clear encoder's internal flag
				
				if (new_count1 != count1) {
					count1 = new_count1;
					while(count1 < 0) count1 += STEPS_PER_REV;
					printf("ENC1 CHANGED: %d ", count1);
					dual_encoder_update();
				}
			}
			
			if(enc2_interrupt) {
				enc2_interrupt = false;  // Clear our flag immediately
				printf("[IRQ2-GPIO27] ");  // Debug info in main loop
				
				int16_t new_count2 = enc2.read();
				enc2.clear_interrupt_flag();  // Clear encoder's internal flag
				
				if (new_count2 != count2) {
					count2 = new_count2;
					while(count2 < 0) count2 += STEPS_PER_REV;
					printf("ENC2 CHANGED: %d ", count2);
					dual_encoder_update();
				}
			}

			// No delay needed with hardware interrupts - they're instant!
			// Just a tiny yield to prevent 100% CPU usage
			
		}
	}
	else {
		printf("✗ No encoders found!\n");
		printf("Check:\n");
		printf("  - Wiring: SDA->GPIO6, SCL->GPIO7\n");
		printf("  - Encoder 1: INT->GPIO26, Address=0x%02X\n", BreakoutEncoder::DEFAULT_I2C_ADDRESS);
		printf("  - Encoder 2: INT->GPIO27, Address=0x0E\n");
		printf("  - Power: VCC and GND connected\n");
		printf("Use 's' command to scan I2C bus\n");
		
#ifdef PICO_DEFAULT_LED_PIN
		printf("Setting onboard LED on to indicate error\n");
		gpio_put(PICO_DEFAULT_LED_PIN, true);
#endif

	
	}

	return 0;
}
