# XIAO RP2040 Dual-Core 4-Encoder WS2812 Animation System

## Overview
This project implements a real-time LED animation system using 4 rotary encoders and a WS2812 LED strip on the Seeed XIAO RP2040. The system uses both cores of the RP2040 for optimal performance.

## Hardware Configuration
- **WS2812 LED Strip**: GPIO 0 (107 LEDs)
- **Encoder 1**: I2C Address 0x0F, IRQ GPIO 1
- **Encoder 2**: I2C Address 0x0E, IRQ GPIO 2  
- **Encoder 3**: I2C Address 0x0D, IRQ GPIO 3
- **Encoder 4**: I2C Address 0x0C, IRQ GPIO 4
- **I2C Bus**: SDA GPIO 6, SCL GPIO 7

## Dual-Core Architecture
- **Core 0**: Handles encoder interrupts, I2C communication, and serial terminal
- **Core 1**: Dedicated to WS2812 LED animations and rendering

## LED Animation Modes

### 1. Built-in Animations
The system includes several preset animations that cycle automatically:
- **Rainbow Cycle**: Smooth rainbow wave across the strip
- **Color Wipe**: Sequential LED color filling
- **Theater Chase**: Moving dot patterns
- **Sparkle**: Random twinkling effects
- **Pulse**: Breathing color effects

### 2. Encoder Control Mode
Interactive mode where encoders control LED parameters in real-time:

#### Encoder Functions:
- **Encoder 1**: Position selector (0-106) - selects which LED pixel to edit
- **Encoder 2**: Color selector - HSV hue value for RGB color selection
- **Encoder 3**: Object size (1-20 pixels) - width of light object with center dimming
- **Encoder 4**: Object management
  - **Positive rotation**: Save current pixel art state to object array, reset position to LED 0
  - **Negative rotation**: Delete the pixel object at current position

#### Pixel Art System:
- **Object Array**: Stores up to 16 pixel art objects
- **Object Storage**: Each object contains position, size, RGB color values, brightness fade pattern
- **Real-time Rendering**: Core 1 continuously renders all stored objects onto the LED strip
- **Interactive Editing**: Live preview while adjusting position, color, and size

## Serial Terminal Commands
- `'n'` - Next animation mode (cycles through built-in animations and encoder mode)
- `'s'` - Scan I2C bus for encoder devices
- `'r'` - Soft reset system
- `'d'` - Debug info (system status, encoder values)
- `'z'` - Reset to bootloader mode
- `'h'` - Show help menu
- `'a1'-'a4'` - Set encoder addresses

## Technical Features

### Real-time Performance
- **Hardware Interrupts**: Instant encoder response on all 4 channels
- **Non-blocking I/O**: No delays in main loops
- **Dual-core Processing**: Parallel encoder and LED processing
- **DMA-based WS2812**: Smooth LED updates without CPU blocking

### Object Rendering System
- **Layered Rendering**: Multiple objects can overlap
- **Center Dimming**: Objects fade from center outward based on size
- **Color Blending**: Overlapping objects blend colors additively
- **Persistent Storage**: Objects remain until explicitly deleted

## Build Instructions
1. Install Pico SDK and CMake
2. Run `.\build_breakout_encoder.ps1`
3. Flash `led_encoder.uf2` to XIAO RP2040 in bootloader mode

## Usage Examples

### Creating Pixel Art:
1. Use command `'n'` to enter encoder control mode
2. Turn **Encoder 1** to select LED position
3. Turn **Encoder 2** to choose color
4. Turn **Encoder 3** to set object size
5. Turn **Encoder 4** positive to save object
6. Repeat to create multiple objects
7. Turn **Encoder 4** negative to delete objects

### Animation Viewing:
1. Use command `'n'` to cycle through built-in animations
2. Encoders show real-time interrupt activity in terminal
3. Each animation runs independently on Core 1
