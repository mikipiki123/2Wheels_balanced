# Generated Cmake Pico project file

# cmake -DPICO_SDK_PATH=/Users/miki/.pico-sdk/sdk/2.2.0 ..

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ... (Keep the VS Code extension block unchanged) ...

# 1. CHANGE THIS: Set board to pico_w to enable CYW43 capabilities
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(2Wheels_balanced C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Create freertos_config INTERFACE target
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config BEFORE INTERFACE SYSTEM
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(freertos_config INTERFACE
    projCOVERAGE_TEST=0
)

# Set FreeRTOS Kernel Path and Import RP2040 Port Helper
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

add_executable(2Wheels_balanced
    src/main.cpp
    src/controller.cpp
    src/hardware.cpp
)

pico_set_program_name(2Wheels_balanced "2Wheels_balanced")
pico_set_program_version(2Wheels_balanced "0.1")

pico_enable_stdio_uart(2Wheels_balanced 1)
pico_enable_stdio_usb(2Wheels_balanced 1)

# 2. ADD pico_cyw43_arch_lwip_sys_freertos to the link libraries
target_link_libraries(2Wheels_balanced PRIVATE
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_gpio
    hardware_clocks

    pico_lwip_mdns
    freertos_config
    FreeRTOS-Kernel-Heap4
    pico_cyw43_arch_lwip_sys_freertos
)

target_include_directories(2Wheels_balanced BEFORE PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(2Wheels_balanced)