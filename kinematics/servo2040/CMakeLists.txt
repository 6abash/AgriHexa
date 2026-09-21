cmake_minimum_required(VERSION 3.12)

set(NAME agrihexa_servo2040)

include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${NAME}
    main.cpp
)

include(drivers/servo/servo.cmake)
include(drivers/servo/servo_cluster.cmake)

target_link_libraries(${NAME}
    pico_stdlib
    hardware_uart
    hardware_pio
    servo
    servo_cluster
)

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)

pico_add_extra_outputs(${NAME})
