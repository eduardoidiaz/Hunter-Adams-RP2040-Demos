# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(Keypad_With_Debouncing C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(Keypad_With_Debouncing)

# enable usb output
pico_enable_stdio_usb(Keypad_With_Debouncing 1)

# must match with executable name and source file names
target_sources(Keypad_With_Debouncing PRIVATE keypad_with_debouncing.c)

# must match with executable name
target_link_libraries(Keypad_With_Debouncing PRIVATE pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pio hardware_dma hardware_sync hardware_spi hardware_clocks)

# must match with executable name
pico_add_extra_outputs(Keypad_With_Debouncing)
