cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME flight)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    commet/config.h
    commet/imu.h
    MDK-ARM/interface/mpu6050.h
    commet/pid.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    commet/comment.c
    commet/filter.c
    commet/imu.c
    commet/pid.c
    MDK-ARM/appliaction/app_freertos_task.c
    MDK-ARM/appliaction/app_flight.c
    MDK-ARM/appliaction/APP_receive_data.c
    MDK-ARM/interface/LED.c
    MDK-ARM/interface/SI24R1.c
    MDK-ARM/interface/tp4336.c
    MDK-ARM/interface/motor.c
    MDK-ARM/interface/mpu6050.c
    
    freertos/croutine.c
    freertos/event_groups.c
    freertos/list.c
    freertos/queue.c
    freertos/stream_buffer.c
    freertos/tasks.c
    freertos/timers.c
    freertos/poarable/heap_4.c
    freertos/poarable/port.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    commet
    freertos/include
    freertos
    freertos/poarable
    MDK-ARM/appliaction
    MDK-ARM/interface
    Core/Inc
)
# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
