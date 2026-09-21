cmake_minimum_required(VERSION 3.13)

set(NAME Hexapod_OS)

include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)

project(${NAME} C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_executable(${NAME} main.cpp)

# Generate the PIO header for the servo cluster driver
pico_generate_pio_header(${NAME} "${PIMORONI_PICO_PATH}/drivers/servo/servo_cluster.pio")

# Pull the servo driver sources in directly (not built as a library target
# in every Pimoroni Pico checkout, so we compile them straight into the app)
target_sources(${NAME} PRIVATE
    "${PIMORONI_PICO_PATH}/drivers/servo/servo_cluster.cpp"
    "${PIMORONI_PICO_PATH}/drivers/servo/servo.cpp"
)

add_subdirectory("${PIMORONI_PICO_PATH}/common" pimoroni_common)
add_subdirectory("${PIMORONI_PICO_PATH}/libraries/servo2040" servo2040)

target_include_directories(${NAME} PUBLIC
    "${PIMORONI_PICO_PATH}"
    "${PIMORONI_PICO_PATH}/drivers/servo"
    "${PIMORONI_PICO_PATH}/drivers/pwm"
)

target_link_libraries(${NAME}
    pico_stdlib
    hardware_pwm
    hardware_dma
    hardware_uart
    hardware_pio
    servo2040
)

pico_enable_stdio_usb(${NAME} 1)
pico_add_extra_outputs(${NAME})

# Workaround for missing PIO functions in Pico SDK 1.5.1
add_definitions(-DPICO_RP2040_USB_DEVICE_UFRAME_FIX=1)
