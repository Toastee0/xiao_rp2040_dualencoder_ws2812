//important coding notes. 
// this is a realtime application, and sleep and printf calls are expensive and only allowed outside of irq functions.
// sleep is to be avoided outside the initial setup of the main loop, and in the case that the bootloader mode has been requested (because we are stopping anyway its ok to sleep now)
// it's important to ensure that the code does not "block" or have any delays, polling must be non-blocking in the same manner as our serial read function
// ws2812 handles led hardware initialization, accepting it's led pin info from main
// the leds should be controlled in an easily removed module so other people can adapt this easily without a led strip to test with.

#include "ws2812_leds.hpp"
#include "main.hpp"
#include "encoder.hpp"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include <stdio.h>
#include <math.h>
#include <cmath>

// ===================================================================
// ================ SYSTEM CONSTANTS AND GLOBALS ===================
// ===================================================================

// Global brightness limiter (0-255, 128 = 50%)
#define GLOBAL_BRIGHTNESS_LIMIT 128

// Use WS2812 configuration from main.hpp
static PIO pio = pio0;
static uint sm = 0;
static bool ws2812_initialized = false;

// LED color buffer - RGB format
static uint32_t led_buffer[NUM_LEDS];

// Animation system state
static AnimationMode current_mode = RAINBOW_CYCLE;
static uint32_t animation_counter = 0;

// Pixel art object storage
static PixelObject pixel_objects[MAX_PIXEL_OBJECTS];
static EncoderControlState encoder_state = {0, 0, 3, 0}; // Start with size 3

// Animation mode names for display
static const char* animation_mode_names[] = {
    "LED OFF",
    "Rainbow Cycle", 
    "Color Wipe",
    "Theater Chase",
    "Sparkle",
    "Pulse",
    "Encoder Control"
};

// Color constants
#define COLOR_BLACK    0x000000
#define COLOR_RED      0xFF0000
#define COLOR_GREEN    0x00FF00
#define COLOR_BLUE     0x0000FF
#define COLOR_WHITE    0xFFFFFF
#define COLOR_YELLOW   0xFFFF00
#define COLOR_CYAN     0x00FFFF
#define COLOR_MAGENTA  0xFF00FF

// ===================================================================
// ================ COLOR UTILITY FUNCTIONS ========================
// ===================================================================

// Convert RGB values to 32-bit color (GRB format for WS2812)
uint32_t rgb_to_grb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
}

// Convert HSV to RGB color space
uint32_t hsv_to_grb(uint16_t h, uint8_t s, uint8_t v) {
    uint8_t r, g, b;
    
    if (s == 0) {
        r = g = b = v;
    } else {
        uint16_t region = h / 43;
        uint16_t remainder = (h - (region * 43)) * 6;
        
        uint8_t p = (v * (255 - s)) >> 8;
        uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
        uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
        
        switch (region) {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            default: r = v; g = p; b = q; break;
        }
    }
    
    return rgb_to_grb(r, g, b);
}

// Blend two colors with alpha
uint32_t blend_colors(uint32_t color1, uint32_t color2, uint8_t alpha) {
    uint8_t r1 = (color1 >> 8) & 0xFF;
    uint8_t g1 = (color1 >> 16) & 0xFF;
    uint8_t b1 = color1 & 0xFF;
    
    uint8_t r2 = (color2 >> 8) & 0xFF;
    uint8_t g2 = (color2 >> 16) & 0xFF;
    uint8_t b2 = color2 & 0xFF;
    
    uint8_t r = ((r1 * (255 - alpha)) + (r2 * alpha)) >> 8;
    uint8_t g = ((g1 * (255 - alpha)) + (g2 * alpha)) >> 8;
    uint8_t b = ((b1 * (255 - alpha)) + (b2 * alpha)) >> 8;
    
    return rgb_to_grb(r, g, b);
}

// Dim a color by a factor
uint32_t dim_color(uint32_t color, uint8_t brightness) {
    uint8_t r = ((color >> 8) & 0xFF) * brightness / 255;
    uint8_t g = ((color >> 16) & 0xFF) * brightness / 255;
    uint8_t b = (color & 0xFF) * brightness / 255;
    
    return rgb_to_grb(r, g, b);
}

// ===================================================================
// ================ LED BUFFER MANAGEMENT ===========================
// ===================================================================

// Clear all LEDs
void clear_leds() {
    for (int i = 0; i < NUM_LEDS; i++) {
        led_buffer[i] = COLOR_BLACK;
    }
}

// Set individual LED color
void set_led_color(uint16_t led_index, uint32_t color) {
    if (led_index < NUM_LEDS) {
        led_buffer[led_index] = color;
    }
}

// Set range of LEDs to same color
void set_led_range(uint16_t start, uint16_t end, uint32_t color) {
    if (start >= NUM_LEDS) return;
    if (end >= NUM_LEDS) end = NUM_LEDS - 1;
    
    for (uint16_t i = start; i <= end; i++) {
        led_buffer[i] = color;
    }
}

// Push LED buffer to hardware (non-blocking) with global brightness limit
void update_leds() {
    if (!ws2812_initialized) return;
    
    for (int i = 0; i < NUM_LEDS; i++) {
        // Apply global brightness limit (50%)
        uint32_t color = led_buffer[i];
        uint8_t r = ((color >> 8) & 0xFF) * GLOBAL_BRIGHTNESS_LIMIT / 255;
        uint8_t g = ((color >> 16) & 0xFF) * GLOBAL_BRIGHTNESS_LIMIT / 255;
        uint8_t b = (color & 0xFF) * GLOBAL_BRIGHTNESS_LIMIT / 255;
        
        uint32_t dimmed_color = rgb_to_grb(r, g, b);
        pio_sm_put_blocking(pio, sm, dimmed_color << 8u);
    }
}

// ===================================================================
// ================ ANIMATION FUNCTIONS =============================
// ===================================================================

// Helper function declarations
static void render_pixel_object(const PixelObject* obj);
static void render_object_at_position(uint16_t position, uint8_t size, uint32_t color, uint8_t brightness);

// Rainbow cycle animation
static void animate_rainbow_cycle() {
    for (int i = 0; i < NUM_LEDS; i++) {
        uint16_t hue = ((i * 256 / NUM_LEDS) + animation_counter) & 0xFF;
        led_buffer[i] = hsv_to_grb(hue, 255, 128);  // Medium brightness
    }
    animation_counter = (animation_counter + 2) & 0xFF;
}

// Encoder position visualization
static void animate_encoder_position() {
    clear_leds();
    
    uint8_t encoder_count = get_encoder_ready_count();
    if (encoder_count == 0) return;
    
    // Divide LED strip into sections for each encoder
    uint16_t leds_per_encoder = NUM_LEDS / encoder_count;
    
    for (uint8_t i = 0; i < encoder_count; i++) {
        const EncoderData* encoder = get_encoder_data(i);
        if (!encoder) continue;
        
        // Calculate position within this encoder's section
        uint16_t section_start = i * leds_per_encoder;
        uint16_t section_end = (i + 1) * leds_per_encoder - 1;
        if (i == encoder_count - 1) section_end = NUM_LEDS - 1;  // Last encoder gets remaining LEDs
        
        // Map encoder position to LED position within section
        int32_t position = encoder->counter;
        uint16_t led_pos = section_start + (abs(position) % (section_end - section_start + 1));
        
        // Choose color based on encoder ID
        uint32_t encoder_color;
        switch (i) {
            case 0: encoder_color = COLOR_RED; break;
            case 1: encoder_color = COLOR_GREEN; break;
            case 2: encoder_color = COLOR_BLUE; break;
            case 3: encoder_color = COLOR_YELLOW; break;
            default: encoder_color = COLOR_WHITE; break;
        }
        
        // Show position with trailing effect
        set_led_color(led_pos, encoder_color);
        if (led_pos > section_start) {
            set_led_color(led_pos - 1, dim_color(encoder_color, 64));
        }
        if (led_pos < section_end) {
            set_led_color(led_pos + 1, dim_color(encoder_color, 64));
        }
    }
}

// Encoder delta (speed) visualization
static void animate_encoder_delta() {
    clear_leds();
    
    uint8_t encoder_count = get_encoder_ready_count();
    if (encoder_count == 0) return;
    
    for (uint8_t i = 0; i < encoder_count; i++) {
        const EncoderData* encoder = get_encoder_data(i);
        if (!encoder) continue;
        
        int32_t delta = encoder->delta;
        if (delta == 0) continue;
        
        // Map delta to LED brightness and color
        uint32_t color = (delta > 0) ? COLOR_GREEN : COLOR_RED;
        uint8_t brightness = (uint8_t)(255 * (abs(delta) > 10 ? 1.0 : abs(delta) / 10.0));
        
        // Light up LEDs based on encoder and delta magnitude
        uint16_t start_led = i * (NUM_LEDS / 4);  // Quarter strip per encoder
        uint16_t num_leds = abs(delta);
        if (num_leds > NUM_LEDS / 4) num_leds = NUM_LEDS / 4;
        
        for (uint16_t j = 0; j < num_leds; j++) {
            set_led_color(start_led + j, dim_color(color, brightness));
        }
    }
}

// Theater chase animation
static void animate_theater_chase() {
    clear_leds();
    
    uint32_t colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};
    uint8_t color_index = (animation_counter / 10) % 3;
    uint8_t phase = animation_counter % 3;
    
    for (int i = phase; i < NUM_LEDS; i += 3) {
        led_buffer[i] = colors[color_index];
    }
    
    animation_counter++;
}

// Solid color mode
static void animate_solid_color() {
    uint32_t color = hsv_to_grb((animation_counter * 2) & 0xFF, 255, 128);
    for (int i = 0; i < NUM_LEDS; i++) {
        led_buffer[i] = color;
    }
    animation_counter++;
}

// Fire effect simulation
static void animate_fire_effect() {
    // Simple fire effect with random flickering
    for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t heat = (animation_counter + i * 3) & 0xFF;
        uint8_t flicker = (heat > 128) ? 255 - heat : heat;
        
        uint8_t r = flicker;
        uint8_t g = flicker > 128 ? flicker - 128 : 0;
        uint8_t b = 0;
        
        led_buffer[i] = rgb_to_grb(r, g, b);
    }
    animation_counter++;
}

// Color wipe animation - fill strip sequentially
static void animate_color_wipe() {
    static uint16_t wipe_position = 0;
    static uint8_t wipe_color_index = 0;
    static uint32_t wipe_colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW, COLOR_CYAN, COLOR_MAGENTA};
    
    uint32_t current_color = wipe_colors[wipe_color_index];
    
    // Fill from 0 to current position
    for (uint16_t i = 0; i <= wipe_position; i++) {
        led_buffer[i] = current_color;
    }
    
    // Clear the rest
    for (uint16_t i = wipe_position + 1; i < NUM_LEDS; i++) {
        led_buffer[i] = COLOR_BLACK;
    }
    
    wipe_position++;
    if (wipe_position >= NUM_LEDS) {
        wipe_position = 0;
        wipe_color_index = (wipe_color_index + 1) % 6;
    }
}

// Sparkle animation - random twinkling
static void animate_sparkle() {
    // Fade all LEDs slightly
    for (int i = 0; i < NUM_LEDS; i++) {
        led_buffer[i] = dim_color(led_buffer[i], 240); // 94% brightness
    }
    
    // Add random sparkles
    if ((animation_counter % 8) == 0) { // Every 8 frames
        uint16_t sparkle_led = animation_counter % NUM_LEDS;
        uint32_t sparkle_color = hsv_to_grb(animation_counter * 13, 255, 255);
        led_buffer[sparkle_led] = sparkle_color;
    }
    
    animation_counter++;
}

// Pulse animation - breathing effect
static void animate_pulse() {
    uint8_t brightness = (uint8_t)(128 + 127 * sin(animation_counter * 0.1));
    uint32_t pulse_color = hsv_to_grb((animation_counter / 4) % 256, 255, brightness);
    
    for (int i = 0; i < NUM_LEDS; i++) {
        led_buffer[i] = pulse_color;
    }
    
    animation_counter++;
}

// Encoder control mode - interactive pixel art editor
static void animate_encoder_control() {
    clear_leds();
    
    // Get current encoder values
    const EncoderData* enc1 = get_encoder_data(0); // Position
    const EncoderData* enc2 = get_encoder_data(1); // Color  
    const EncoderData* enc3 = get_encoder_data(2); // Size
    const EncoderData* enc4 = get_encoder_data(3); // Save/Delete
    
    if (enc1) {
        // Update position (0 to NUM_LEDS-1)
        encoder_state.current_position = ((enc1->counter % NUM_LEDS) + NUM_LEDS) % NUM_LEDS;
    }
    
    if (enc2) {
        // Update color hue (0-255)
        encoder_state.current_hue = ((enc2->counter * 8) % 256 + 256) % 256;
    }
    
    if (enc3) {
        // Update size (1-20 pixels)
        int32_t size = enc3->counter;
        if (size < 1) size = 1;
        if (size > 20) size = 20;
        encoder_state.current_size = (uint8_t)size;
    }
    
    if (enc4) {
        // Handle save/delete on encoder 4 delta
        int32_t delta4 = enc4->counter - encoder_state.last_encoder4_value;
        if (delta4 > 0) {
            // Positive rotation - save object and reset position
            save_current_pixel_object();
            encoder_state.current_position = 0;
        } else if (delta4 < 0) {
            // Negative rotation - delete object at current position
            delete_pixel_object_at_position(encoder_state.current_position);
        }
        encoder_state.last_encoder4_value = enc4->counter;
    }
    
    // Render all saved pixel objects
    for (int i = 0; i < MAX_PIXEL_OBJECTS; i++) {
        if (pixel_objects[i].active) {
            render_pixel_object(&pixel_objects[i]);
        }
    }
    
    // Render current editing object (preview)
    uint32_t preview_color = hsv_to_grb(encoder_state.current_hue, 255, 128); // Dimmer for preview
    render_object_at_position(encoder_state.current_position, encoder_state.current_size, preview_color, 128);
}

// Helper function to render a pixel object
static void render_pixel_object(const PixelObject* obj) {
    uint32_t color = rgb_to_grb(obj->red, obj->green, obj->blue);
    render_object_at_position(obj->position, obj->size, color, obj->brightness);
}

// Helper function to render an object at a specific position with fading
static void render_object_at_position(uint16_t position, uint8_t size, uint32_t color, uint8_t brightness) {
    int16_t half_size = size / 2;
    
    for (int16_t offset = -half_size; offset <= half_size; offset++) {
        int16_t led_pos = position + offset;
        
        // Handle wrapping
        if (led_pos < 0) led_pos += NUM_LEDS;
        if (led_pos >= NUM_LEDS) led_pos -= NUM_LEDS;
        
        // Calculate fade based on distance from center
        uint8_t fade = 255;
        if (size > 1) {
            fade = 255 - (abs(offset) * 255 / half_size);
        }
        
        uint8_t final_brightness = (brightness * fade) / 255;
        uint32_t faded_color = dim_color(color, final_brightness);
        
        // Blend with existing color (for overlapping objects)
        led_buffer[led_pos] = blend_colors(led_buffer[led_pos], faded_color, final_brightness);
    }
}

// ===================================================================
// ================ PUBLIC API IMPLEMENTATION ======================
// ===================================================================

bool init_ws2812_system() {
    printf("Initializing WS2812 LED system...\n");
    
    // Load PIO program
    uint offset = pio_add_program(pio, &ws2812_program);
    
    // Initialize state machine manually
    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, WS2812_PIN);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    
    pio_gpio_init(pio, WS2812_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, WS2812_PIN, 1, true);
    
    float div = (float)clock_get_hz(clk_sys) / (800000 * 3.0);  // 800kHz * 3 cycles per bit
    sm_config_set_clkdiv(&c, div);
    
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    
    // Clear all LEDs initially
    clear_leds();
    update_leds();
    
    ws2812_initialized = true;
    animation_counter = 0;
    
    printf("WS2812 system initialized on GPIO %d\n", WS2812_PIN);
    return true;
}

// ===================================================================
// ================ ANIMATION MODE CONTROL ==========================
// ===================================================================

void next_animation_mode() {
    current_mode = (AnimationMode)((current_mode + 1) % 7); // 7 total modes
    animation_counter = 0; // Reset animation counter
    clear_leds(); // Clear screen when switching modes
    
    // Initialize encoder control mode if selected
    if (current_mode == ENCODER_CONTROL) {
        encoder_state.current_position = 0;
        encoder_state.current_hue = 0;
        encoder_state.current_size = 3;
        encoder_state.last_encoder4_value = 0;
    }
}

AnimationMode get_current_animation_mode() {
    return current_mode;
}

const char* get_animation_mode_name() {
    return animation_mode_names[current_mode];
}

// ===================================================================
// ================ PIXEL OBJECT MANAGEMENT =========================
// ===================================================================

void save_current_pixel_object() {
    // Find an empty slot
    for (int i = 0; i < MAX_PIXEL_OBJECTS; i++) {
        if (!pixel_objects[i].active) {
            pixel_objects[i].active = true;
            pixel_objects[i].position = encoder_state.current_position;
            pixel_objects[i].size = encoder_state.current_size;
            
            // Convert HSV to RGB
            uint32_t color = hsv_to_grb(encoder_state.current_hue, 255, 255);
            pixel_objects[i].red = (color >> 8) & 0xFF;
            pixel_objects[i].green = (color >> 16) & 0xFF;
            pixel_objects[i].blue = color & 0xFF;
            pixel_objects[i].brightness = 255;
            
            printf("Saved object %d: pos=%d, size=%d, color=HSV(%d)\n", 
                   i, encoder_state.current_position, encoder_state.current_size, encoder_state.current_hue);
            return;
        }
    }
    printf("Object storage full (max %d objects)\n", MAX_PIXEL_OBJECTS);
}

void delete_pixel_object_at_position(uint16_t position) {
    for (int i = 0; i < MAX_PIXEL_OBJECTS; i++) {
        if (pixel_objects[i].active && pixel_objects[i].position == position) {
            pixel_objects[i].active = false;
            printf("Deleted object %d at position %d\n", i, position);
            return;
        }
    }
    printf("No object found at position %d\n", position);
}

void clear_all_pixel_objects() {
    for (int i = 0; i < MAX_PIXEL_OBJECTS; i++) {
        pixel_objects[i].active = false;
    }
    printf("All pixel objects cleared\n");
}

uint8_t get_active_object_count() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_PIXEL_OBJECTS; i++) {
        if (pixel_objects[i].active) count++;
    }
    return count;
}

void update_ws2812_system() {
    if (!ws2812_initialized) return;
    
    // Run current animation
    switch (current_mode) {
        case RAINBOW_CYCLE:
            animate_rainbow_cycle();
            break;
        case COLOR_WIPE:
            animate_color_wipe();
            break;
        case THEATER_CHASE:
            animate_theater_chase();
            break;
        case SPARKLE:
            animate_sparkle();
            break;
        case PULSE:
            animate_pulse();
            break;
        case ENCODER_CONTROL:
            animate_encoder_control();
            break;
        case LED_OFF:
        default:
            clear_leds();
            break;
    }
    
    // Update LED hardware
    update_leds();
}

void set_animation_mode(AnimationMode mode) {
    current_mode = mode;
    animation_counter = 0;  // Reset animation counter
    printf("Animation mode: %s\n", animation_mode_names[mode]);
}

void set_all_leds_color(uint32_t color) {
    for (int i = 0; i < NUM_LEDS; i++) {
        led_buffer[i] = color;
    }
    update_leds();
}

void set_led_brightness(uint8_t brightness) {
    // Apply brightness to entire buffer
    for (int i = 0; i < NUM_LEDS; i++) {
        led_buffer[i] = dim_color(led_buffer[i], brightness);
    }
}

bool is_ws2812_ready() {
    return ws2812_initialized;
}

void test_led_pattern() {
    printf("Testing LED pattern...\n");
    
    // Test red
    set_all_leds_color(COLOR_RED);
    update_leds();
    sleep_ms(500);  // Test function allows sleep
    
    // Test green
    set_all_leds_color(COLOR_GREEN);
    update_leds();
    sleep_ms(500);
    
    // Test blue
    set_all_leds_color(COLOR_BLUE);
    update_leds();
    sleep_ms(500);
    
    // Clear
    clear_leds();
    update_leds();
    
    printf("LED test complete\n");
}
