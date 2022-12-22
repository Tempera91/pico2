# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/simone/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(log_caricabatterie C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(log_caricabatterie log_caricabatterie.c )

pico_set_program_name(log_caricabatterie "log_caricabatterie")
pico_set_program_version(log_caricabatterie "0.1")

pico_enable_stdio_uart(log_caricabatterie 0)
pico_enable_stdio_usb(log_caricabatterie 1)

# Add the standard library to the build
target_link_libraries(log_caricabatterie pico_stdlib)

# Add any user requested libraries
target_link_libraries(log_caricabatterie
        hardware_timer
        hardware_clocks
        hardware_gpio
        hardware_adc
        )

pico_add_extra_outputs(log_caricabatterie)

