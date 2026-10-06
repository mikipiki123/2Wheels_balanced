# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(2Wheels_balanced C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# 1. Create freertos_config INTERFACE target
add_library(freertos_config INTERFACE)

# Syntax fix: 'INTERFACE' must come before 'SYSTEM'
target_include_directories(freertos_config BEFORE INTERFACE SYSTEM
    ${CMAKE_CURRENT_SOURCE_DIR}/include # Directory containing FreeRTOSConfig.h
)

target_compile_definitions(freertos_config INTERFACE
    projCOVERAGE_TEST=0
)

# 2. Set FreeRTOS Kernel Path and Import RP2040 Port Helper
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# 3. Add executable target
add_executable(2Wheels_balanced
    src/main.cpp
    src/controller.cpp
    src/hardware.cpp
)

pico_set_program_name(2Wheels_balanced "2Wheels_balanced")
pico_set_program_version(2Wheels_balanced "0.1")

# Enable stdio output over UART/USB
pico_enable_stdio_uart(2Wheels_balanced 1)
pico_enable_stdio_usb(2Wheels_balanced 1)

# 4. Link Libraries
target_link_libraries(2Wheels_balanced PRIVATE
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_gpio
    hardware_clocks

    freertos_config
    FreeRTOS-Kernel-Heap4
)

# 5. Include Directories
target_include_directories(2Wheels_balanced BEFORE PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(2Wheels_balanced)