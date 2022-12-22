# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/simone/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(stazione_meteo C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(stazione_meteo stazione_meteo.c
                aht20_lib.c)

pico_set_program_name(stazione_meteo "stazione_meteo")
pico_set_program_version(stazione_meteo "0.1")

pico_enable_stdio_uart(stazione_meteo 0)
pico_enable_stdio_usb(stazione_meteo 1)

# Add the standard library to the build
target_link_libraries(stazione_meteo pico_stdlib
        pico_multicore
        )

# Add any user requested libraries
target_link_libraries(stazione_meteo
        hardware_spi
        hardware_i2c
        hardware_timer
        hardware_clocks
        hardware_sync
        )

pico_add_extra_outputs(stazione_meteo)

