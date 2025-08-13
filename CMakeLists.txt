
cmake_minimum_required(VERSION 3.13)

# Set board type to Seeed XIAO RP2040
set(PICO_BOARD seeed_xiao_rp2040)

# Set the PICO_SDK_PATH
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../pico-sdk")

# Pull in Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

project(breakout_encoder_project)

# Initialize the SDK
pico_sdk_init()

add_executable(led_encoder
	led_encoder.cpp
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
)

# Enable usb output, disable uart output
pico_enable_stdio_usb(led_encoder 1)
pico_enable_stdio_uart(led_encoder 1)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(led_encoder)
