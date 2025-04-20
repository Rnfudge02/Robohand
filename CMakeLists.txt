# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

#
# @file CMakeLists.txt
# @brief Build configuration for the robotic hand control system.
# @details Sets up targets for different connection backends (USB, Micro-ROS).
# @author Robert Fudge
# @date 2025
# @copyright Apache 2.0 License
#

# Set the minimum CMake version allowed for compilation
cmake_minimum_required(VERSION 3.13)

# Default board type (can be overridden via command line)
set(PICO_BOARD pico CACHE STRING "Board type")

# Standard language settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Project structure configuration
set(PROJECT_NAME Robohand)
set(DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Include)

# Pico SDK configuration
set(PICO_SDK_PATH ${DEPS_DIR}/pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Micro-ROS configuration
set(MICRO_ROS_DIR ${DEPS_DIR}/micro_ros_pico_sdk)
set(MICRO_ROS_PLATFORM_DIR ${MICRO_ROS_DIR}/libmicroros)

# Build options with descriptions
option(USE_INTERRUPTS "Enable interrupt-based data acquisition" OFF)
option(USE_DMA "Enable DMA transfers for sensor data" OFF)
option(ENABLE_DEBUG "Enable detailed debug output" ON)
option(ENABLE_WATCHDOG "Enable watchdog timer for system recovery" ON)

# Main project setup
project(${PROJECT_NAME} C CXX ASM)

# Initialize Pico SDK
pico_sdk_init()
set(PICO_LWIP_COMPILER_FLAGS "-Wno-address-of-packed-member -Wno-unused-but-set-variable")

# Common compile definitions for all targets
set(COMMON_DEFINITIONS
    PICO_DEFAULT_CPU_CLOCK_KHZ=125000
    PICO_FLASH_SPI_CLKDIV=4
    PICO_NO_FLASH=0
)

# Additional definitions based on build options
if(USE_DMA)
    list(APPEND COMMON_DEFINITIONS
        USE_DMA=1
        PICO_DEFAULT_DMA_MAX_CHANNELS=12
    )
endif()

if(USE_INTERRUPTS)
    list(APPEND COMMON_DEFINITIONS
        USE_INTERRUPTS=1
    )
endif()

if(ENABLE_DEBUG)
    list(APPEND COMMON_DEFINITIONS
        DEBUG=1
    )
else()
    list(APPEND COMMON_DEFINITIONS
        DEBUG=0
    )
endif()

if(ENABLE_WATCHDOG)
    list(APPEND COMMON_DEFINITIONS
        WATCHDOG_ENABLED=1
    )
endif()

# Common source files used by all build targets
set(COMMON_SOURCES
    ./Include/Robohand_common.h
    ./Src/Core/Robohand_callbacks.c
    ./Include/Robohand_callbacks.h
    ./Src/Core/Robohand_dma.c
    ./Include/Robohand_dma.h
    ./Src/Core/Robohand_i2c.c
    ./Include/Robohand_i2c.h
    ./Src/Core/Robohand_init.c
    ./Include/Robohand_init.h
    ./Src/Core/Robohand_interrupts.c
    ./Include/Robohand_interrupts.h
    ./Src/Core/Robohand_reader.c
    ./Include/Robohand_reader.h
    ./Src/Core/Robohand_rgb.c
    ./Include/Robohand_rgb.h
    ./Src/Core/Robohand_servos.c
    ./Include/Robohand_servos.h
    ./Src/Core/Robohand_struct.c
    ./Include/Robohand_struct.h
    ./Src/Core/Robohand_timing.c
    ./Include/Robohand_timing.h
    ./Src/Robohand.c
    ./Include/Robohand.h
)

# Set include directories for all targets
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    ${MICRO_ROS_PLATFORM_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Configure imported Micro-ROS library
add_library(microros STATIC IMPORTED)
set_target_properties(microros PROPERTIES
    IMPORTED_LOCATION ${MICRO_ROS_PLATFORM_DIR}/libmicroros.a
    INTERFACE_INCLUDE_DIRECTORIES "${MICRO_ROS_PLATFORM_DIR}/include"
)

# Common libraries required by all targets
set(COMMON_LIBS
    pico_multicore
    pico_stdlib
    pico_time
    hardware_adc
    hardware_dma
    hardware_i2c
    hardware_pio
    hardware_pwm
    hardware_watchdog
)

# Build variants configuration (target suffix : board type)
set(TARGET_VARIANTS
    "_uros:pico"
    "_usb:pico"
)

# Configure each executable variant
foreach(TARGET_INFO IN LISTS TARGET_VARIANTS)
    string(REPLACE ":" ";" TARGET_PARTS ${TARGET_INFO})
    list(GET TARGET_PARTS 0 TARGET_SUFFIX)
    list(GET TARGET_PARTS 1 PICO_BOARD)

    set(TARGET_NAME ${PROJECT_NAME}${TARGET_SUFFIX})
    set(SOURCE_FILE ./Src/Robohand${TARGET_SUFFIX}.c)

    # Handle special case for Micro-ROS target
    if(TARGET_SUFFIX STREQUAL "_uros")
        set(SOURCE_FILE ./Src/Robohand${TARGET_SUFFIX}.c ./Dependencies/micro_ros_pico_sdk/pico_uart_transport.c)
    endif()

    # Create executable target
    add_executable(${TARGET_NAME}
        ${SOURCE_FILE}
        ${COMMON_SOURCES}
    )

    # Configure target board
    set_target_properties(${TARGET_NAME} PROPERTIES 
        PICO_BOARD ${PICO_BOARD}
    )

    # Apply common compile definitions
    target_compile_definitions(${TARGET_NAME} PRIVATE ${COMMON_DEFINITIONS})

    # Set program metadata
    pico_set_program_name(${TARGET_NAME} "${TARGET_NAME}")
    pico_set_program_version(${TARGET_NAME} "0.1")

    # Link common libraries
    target_link_libraries(${TARGET_NAME} ${COMMON_LIBS})

    # Apply target-specific configuration

    # USB interface target configuration
    if(TARGET_SUFFIX STREQUAL "_usb")
        pico_enable_stdio_usb(${TARGET_NAME} 1)
        pico_enable_stdio_uart(${TARGET_NAME} 0)
        target_compile_definitions(${TARGET_NAME} PRIVATE
            PICO_DEFAULT_USB_MANUFACTURER="Robert Fudge"
            PICO_DEFAULT_USB_PRODUCT="Robohand USB interface"
            PICO_TINYUSB_ENABLE_CDC=1
            # Fix for terminal echo and backspace
            PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
            PICO_STDIO_USB_ENABLE_STDIO_UART_BUILDING=0
            PICO_STDIO_DEFAULT_CRLF=1
        )

        target_link_libraries(${TARGET_NAME} pico_stdio_usb)
    endif()

    # Micro-ROS target configuration
    if(TARGET_SUFFIX STREQUAL "_uros")
        target_link_libraries(${TARGET_NAME} microros)
        target_include_directories(${TARGET_NAME} PRIVATE
            ${MICRO_ROS_PLATFORM_DIR}
            ${MICRO_ROS_PLATFORM_DIR}/include
        )

        # Disable CRLF handling for Micro-ROS
        target_compile_definitions(${TARGET_NAME} PRIVATE
            PICO_UART_ENABLE_CRLF_SUPPORT=0
            PICO_STDIO_ENABLE_CRLF_SUPPORT=0
            PICO_STDIO_DEFAULT_CRLF=0
        )
    endif()

    # Generate UF2 and other output formats
    pico_add_extra_outputs(${TARGET_NAME})
endforeach()

# Helper target for IDE integration
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    add_custom_target(copy_compile_commands ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_SOURCE_DIR}/compile_commands.json
        COMMENT "Copying compile commands for IDE support..."
    )
endif()