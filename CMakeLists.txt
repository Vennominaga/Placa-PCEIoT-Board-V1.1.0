# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

cmake_minimum_required(VERSION 3.13)

set(PICO_FREERTOS_PATH "${PICO_SDK_PATH}/lib/FreeRTOS-Kernel")

# Pull in FreeRTOS
#include(FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
include(${PICO_SDK_PATH}/lib/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(main main.c opt4001.c Register_ICM42670P.c icm42670p.c register_sx1509b.c sx1509b.c sx1509b_irq_manager.c)

target_include_directories(main PRIVATE ${CMAKE_CURRENT_LIST_DIR})
 
# pull in common dependencies
target_link_libraries(main pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap4 hardware_i2c hardware_spi)

pico_enable_stdio_uart(main 0)
pico_enable_stdio_usb(main 1)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)
