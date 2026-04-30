cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "Set PICO_SDK_PATH to the Raspberry Pi Pico SDK")
endif()

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2 CACHE STRING "PICO target board")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_j2534_firmware C CXX ASM)
pico_sdk_init()

add_executable(pico_j2534
    src/main.c
    src/mcp2515.c
    src/usb_descriptors.c
)

target_include_directories(pico_j2534 PRIVATE include)
set(PICOJ_MCP2515_OSC_HZ "" CACHE STRING "MCP2515 crystal frequency in Hz, usually 8000000 or 16000000")
if(PICOJ_MCP2515_OSC_HZ)
    target_compile_definitions(pico_j2534 PRIVATE PICOJ_MCP2515_OSC_HZ=${PICOJ_MCP2515_OSC_HZ})
endif()
target_link_libraries(pico_j2534 PRIVATE
    pico_stdlib
    hardware_spi
    tinyusb_device
    tinyusb_board
)

pico_enable_stdio_usb(pico_j2534 0)
pico_enable_stdio_uart(pico_j2534 0)
pico_add_extra_outputs(pico_j2534)
