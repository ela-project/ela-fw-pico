cmake_minimum_required(VERSION 3.16)

# set(CMAKE_VERBOSE_MAKEFILE 1)

include(lib/pico-sdk/pico_sdk_init.cmake)

project(ela-fw-pico C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

pico_sdk_init()

add_subdirectory(lib/etl)

add_executable(${PROJECT_NAME}
    src/main.cpp
    lib/ela-protocol-lib/src/ela_protocol.c
    src/analyzer.cpp
    src/comms.cpp
    src/hardware.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    src
    src/pico
    lib/ela-protocol-lib/src
)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--print-memory-usage")

target_link_libraries(${PROJECT_NAME}
    etl
    pico_stdlib
    pico_stdio_usb
    pico_multicore
    hardware_pwm
    hardware_dma
    hardware_pio
    hardware_clocks
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/pins_input.pio)

pico_add_extra_outputs(${PROJECT_NAME})

add_custom_command(
    TARGET ${PROJECT_NAME}
    COMMAND "arm-none-eabi-objdump"
    ARGS "-S" "${PROJECT_NAME}.elf" ">" "${PROJECT_NAME}.lst"
)
