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

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# -Wno-unused-parameterを無視する
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

# micro-ROS (optional but enabled when present)
set(MICROROS_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/microros/include" CACHE PATH "micro-ROS include directory (e.g. .../include)")
set(MICROROS_LIB "${CMAKE_SOURCE_DIR}/microros/libmicroros.a" CACHE FILEPATH "micro-ROS static library (e.g. .../libmicroros.a)")


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME f446_hal_microros)

# Include toolchain file
include("${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")

# Target CPU / FPU flags (ensure all objects use the same ABI).
set(STM32_CPU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${STM32_CPU_FLAGS}
    -ffunction-sections
    -fdata-sections
)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${STM32_CPU_FLAGS}
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS Src/*.c Src/*.cpp)

# Enabled libraries under Lib/ (semicolon-separated), or "ALL" for everything.
set(F446_LIBS "ALL" CACHE STRING "Lib modules under Lib/ to build (semicolon-separated), or ALL")
set(USER_LIB_SOURCES "")
if(F446_LIBS STREQUAL "ALL")
    file(GLOB_RECURSE USER_LIB_SOURCES CONFIGURE_DEPENDS Lib/*/Src/*.c Lib/*/Src/*.cpp)
else()
    foreach(lib ${F446_LIBS})
        file(GLOB_RECURSE ONE_LIB_SOURCES CONFIGURE_DEPENDS "Lib/${lib}/Src/*.c" "Lib/${lib}/Src/*.cpp")
        list(APPEND USER_LIB_SOURCES ${ONE_LIB_SOURCES})
    endforeach()
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${USER_SOURCES}
    ${USER_LIB_SOURCES}
)

# micro-ROS include/lib for Lib/MicroRos module headers (e.g., <rcl/rcl.h>)
set(F446_ENABLE_MICROROS OFF)
if(EXISTS "${CMAKE_SOURCE_DIR}/Lib/MicroRos" OR EXISTS "${CMAKE_SOURCE_DIR}/microros")
    set(F446_ENABLE_MICROROS ON)
endif()

if(F446_ENABLE_MICROROS)
    if(NOT EXISTS "${MICROROS_INCLUDE_DIR}")
        message(FATAL_ERROR "micro-ROS is enabled but MICROROS_INCLUDE_DIR does not exist: ${MICROROS_INCLUDE_DIR}\nBuild it with: ./scripts/build_microros_lib.sh")
    endif()
    if(NOT EXISTS "${MICROROS_LIB}")
        message(FATAL_ERROR "micro-ROS is enabled but MICROROS_LIB does not exist: ${MICROROS_LIB}\nBuild it with: ./scripts/build_microros_lib.sh")
    endif()

    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${MICROROS_INCLUDE_DIR}
    )
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
        ${MICROROS_LIB}
    )
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        F446_USE_MICROROS=1
    )
endif()

# Add include paths
file(GLOB USER_LIB_DIRS Lib/*/Inc)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Inc
    ${USER_LIB_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    stm32cubemx
    # Add user defined libraries
)
