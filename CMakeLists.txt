
cmake_minimum_required(VERSION 3.13)
# add a project directive


# Set board type to Seeed XIAO RP2040
set(PICO_BOARD seeed_xiao_rp2040)

# Set the PICO_SDK_PATH
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../pico-sdk")

# Set picotool path for MSYS2/Unix environment on Windows
set(PICOTOOL_EXECUTABLE "/c/picoprojects/tools/bin/picotool.exe")
set(picotool_FOUND TRUE)
set(PICOTOOL_FOUND TRUE)
# Alternative paths in case the above doesn't work
if(NOT EXISTS ${PICOTOOL_EXECUTABLE})
    set(PICOTOOL_EXECUTABLE "C:/picoprojects/tools/bin/picotool.exe")
endif()

# Pull in Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

project(breakout_encoder_project)

# Initialize the SDK
pico_sdk_init()

add_executable(led_encoder
	main.cpp
	encoder.cpp
	ws2812_leds.cpp
)

# Generate PIO header
pico_generate_pio_header(led_encoder ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# Add include directory for pimoroni-pico root to support relative includes
target_include_directories(led_encoder PRIVATE "../pimoroni-pico")

# Add Pimoroni library subdirectories
#add_subdirectory("../pimoroni-pico/drivers/ioexpander" ioexpander_lib)
add_subdirectory("../pimoroni-pico/libraries/breakout_encoder" breakout_encoder_lib)
add_subdirectory("../pimoroni-pico/common" pimoroni_common_lib)

target_link_libraries(led_encoder
	pico_stdlib
	pico_bootrom
	breakout_encoder
	hardware_pio
	hardware_i2c
	hardware_gpio
	hardware_irq
)

# Enable usb output, disable uart output (DO NOT REMOVE)
pico_enable_stdio_usb(led_encoder 1)
pico_enable_stdio_uart(led_encoder 0)

# Create map/bin/hex/uf2 file etc. (DO NOT REMOVE)
pico_add_extra_outputs(led_encoder)
