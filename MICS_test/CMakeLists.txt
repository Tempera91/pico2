# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/simone/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(MICS_test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(MICS_test MICS_test.c
        Pico_hw.c 
        MICS_RSSI.c 
        MICS_util.c
        MICS_comm.c
        CC_hw.c )

pico_set_program_name(MICS_test "MICS_test")
pico_set_program_version(MICS_test "0.1")

pico_enable_stdio_uart(MICS_test 0)
pico_enable_stdio_usb(MICS_test 1)

target_sources(MICS_test PUBLIC MICS_reg_man.c)
# Add the standard library to the build
target_link_libraries(MICS_test pico_stdlib)

# Add any user requested libraries
target_link_libraries(MICS_test
        hardware_spi
        hardware_timer
        hardware_adc
        pico_malloc
        )

pico_add_extra_outputs(MICS_test)

