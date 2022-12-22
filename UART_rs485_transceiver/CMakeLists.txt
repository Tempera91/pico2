# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/simone/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(UART_rs485_transceiver C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(UART_rs485_transceiver UART_rs485_transceiver.c )

pico_set_program_name(UART_rs485_transceiver "UART_rs485_transceiver")
pico_set_program_version(UART_rs485_transceiver "0.1")

pico_enable_stdio_uart(UART_rs485_transceiver 0)
pico_enable_stdio_usb(UART_rs485_transceiver 1)

# Add the standard library to the build
target_link_libraries(UART_rs485_transceiver pico_stdlib)

# Add any user requested libraries
target_link_libraries(UART_rs485_transceiver
        hardware_i2c
        hardware_timer
        pico_runtime
        pico_stdio_usb
        )

pico_add_extra_outputs(UART_rs485_transceiver)

