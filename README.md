# XIAO RP2040 Multi-Encoder LED Animation System

A scalable encoder system for the Seeed XIAO RP2040 that supports 1-4 rotary encoders with WS2812 LED strip animation.

## Pin Configuration

### Fixed Pins
- **I2C**: SDA=GPIO6, SCL=GPIO7 (XIAO RP2040 default)
- **WS2812 LED Strip**: GPIO1
- **LED Count**: 117 LEDs (configurable in code)

### Encoder Pins
| Encoder | GPIO Pin | I2C Address |
|---------|----------|-------------|
| 1       | GPIO26   | 0x0F        |
| 2       | GPIO27   | 0x0E        |
| 3       | GPIO0    | 0x0D        |
| 4       | GPIO29   | 0x0C        |

## Configuration

### Number of Encoders
Edit `led_encoder.cpp` and change:
```cpp
#define NUMBER_OF_ENCODERS 2  // Set to 1, 2, 3, or 4
```

### LED Animation Modes
- **1 Encoder**: Single white ball with trails
- **2 Encoders**: Dual colored balls (cyan/magenta)
- **3-4 Encoders**: Multi-colored balls (red/green/blue/yellow)

## USB Terminal Commands

Connect via USB serial terminal (115200 baud):

### Address Configuration
- `a1` - Set encoder 1 to address 0x0F
- `a2` - Set encoder 2 to address 0x0E  
- `a3` - Set encoder 3 to address 0x0D
- `a4` - Set encoder 4 to address 0x0C

### System Commands
- `s` - Scan I2C bus for connected devices
- `h` - Show help menu
- `z` - Reset to bootloader mode (BOOTSEL)

## Hardware Setup

1. **Connect I2C**: Breakout encoder SDA→GPIO6, SCL→GPIO7
2. **Connect Interrupts**: Encoder INT pin to corresponding GPIO (26-29)
3. **Connect Power**: VCC and GND to encoder boards
4. **Connect LED Strip**: WS2812 data pin to GPIO1
5. **Set Addresses**: Use `a1`-`a4` commands to configure encoder addresses

## Build

```bash
./build_breakout_encoder.ps1
```

## Features

- ✅ Hardware interrupt-driven (no polling)
- ✅ Configurable 1-4 encoder support
- ✅ Real-time LED animation with persistence trails
- ✅ Non-blocking main loop for maximum responsiveness
- ✅ USB terminal configuration interface
- ✅ Automatic encoder detection and initialization
