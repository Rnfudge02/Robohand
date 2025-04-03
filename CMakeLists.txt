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
set(PICO_BOARD pico CACHE STRING "Board type")

#
# @file CMakeLists.txt
# @brief Implementation of robotic hand control system with dual-core operation.
# @details Manages hardware interfaces, sensor data collection, and servo control on Core 1.
# @author Robert Fudge
# @date 2025
# @copyright Apache 2.0 License
#

#Set the minimum CMake version allowed for compilation
cmake_minimum_required(VERSION 3.13)

#Base configuration
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#Project structure configuration
set(PROJECT_NAME Robohand)
set(DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Include)

#Pico SDK configuration
set(PICO_SDK_PATH ${DEPS_DIR}/pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

#Micro-ROS configuration
set(MICRO_ROS_DIR ${DEPS_DIR}/micro_ros_pico_sdk)
set(MICRO_ROS_PLATFORM_DIR ${MICRO_ROS_DIR}/libmicroros)

option(USE_INTERRUPTS "Enable DMA transfers" OFF)
option(USE_DMA "Enable DMA transfers" OFF)

#Main project setup
project(${PROJECT_NAME} C CXX ASM)

#Initialize Pico SDK
pico_sdk_init()
set(PICO_LWIP_COMPILER_FLAGS "-Wno-address-of-packed-member -Wno-unused-but-set-variable")

#Common sources and includes
set(COMMON_SOURCES
    ./Src/Robohand.c
    ./Src/Robohand_advanced.c
    ./Include/Robohand.h
    ./Include/Robohand_advanced.h
)

#Set include directories that header files can be found in
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    ${MICRO_ROS_PLATFORM_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

#Link the libmicroros library
add_library(microros STATIC IMPORTED)
set_target_properties(microros PROPERTIES
    IMPORTED_LOCATION ${MICRO_ROS_PLATFORM_DIR}/libmicroros.a
    INTERFACE_INCLUDE_DIRECTORIES "${MICRO_ROS_PLATFORM_DIR}/include"
)

#Build variations
set(TARGET_VARIANTS
    "_uros:pico"
    "_usb:pico"
)

#Configure for each executable
foreach(TARGET_INFO IN LISTS TARGET_VARIANTS)
    string(REPLACE ":" ";" TARGET_PARTS ${TARGET_INFO})
    list(GET TARGET_PARTS 0 TARGET_SUFFIX)
    list(GET TARGET_PARTS 1 PICO_BOARD)

    set(TARGET_NAME ${PROJECT_NAME}${TARGET_SUFFIX})
    set(SOURCE_FILE ./Src/Robohand${TARGET_SUFFIX}.c)

    # Create executable first
    add_executable(${TARGET_NAME} 
        ${SOURCE_FILE} 
        ${COMMON_SOURCES}
    )

    # THEN configure target properties
    set_target_properties(${TARGET_NAME} PROPERTIES 
        PICO_BOARD ${PICO_BOARD}
    )

    # Common configurations for all targets
    target_compile_definitions(${TARGET_NAME} PRIVATE
        PICO_BOARD_${PICO_BOARD}
        PICO_DEFAULT_CPU_CLOCK_KHZ=125000
        PICO_FLASH_SPI_CLKDIV=4
        PICO_NO_FLASH=0
    )

    # DMA-specific configuration

    if(USE_DMA)
        target_compile_definitions(${TARGET_NAME} PRIVATE
        USE_DMA=1
        PICO_DMA_ALIGN=4
        PICO_DEFAULT_DMA_MAX_CHANNELS=12
        PICO_USE_DMA_RAM=1
        )
        target_link_libraries(${TARGET_NAME}
        hardware_dma
        )
endif()

    # Continue with other configurations...
    pico_set_program_name(${TARGET_NAME} "${TARGET_NAME}")
    pico_set_program_version(${TARGET_NAME} "0.1")

    #Common libraries for all targets
    target_link_libraries(${TARGET_NAME}
        pico_stdlib
        pico_time
        pico_multicore
        hardware_watchdog
        hardware_i2c
        hardware_adc
        hardware_dma
        hardware_pwm
        hardware_pio
    )

    #USB-specific configuration
    if(TARGET_SUFFIX STREQUAL "_usb")
        pico_enable_stdio_usb(${TARGET_NAME} 1)
        pico_enable_stdio_uart(${TARGET_NAME} 0)
        target_compile_definitions(${TARGET_NAME} PRIVATE
            PICO_DEFAULT_USB_MANUFACTURER="Robohand Inc."
            PICO_DEFAULT_USB_PRODUCT="Robotic Hand Controller"
            PICO_TINYUSB_ENABLE_CDC=1
        )
        target_link_libraries(${TARGET_NAME}
            pico_stdio_usb
        )
    endif()

    #Micro-ROS specific configuration
    if(TARGET_SUFFIX MATCHES "_uros")
        target_link_libraries(${TARGET_NAME} microros)
        target_include_directories(${TARGET_NAME} PRIVATE
            ${MICRO_ROS_PLATFORM_DIR}/include
        )
    endif()

    #Common compile definitions
    target_compile_definitions(${TARGET_NAME} PRIVATE
        PICO_UART_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_DEFAULT_CRLF=0
    )

    pico_add_extra_outputs(${TARGET_NAME})
endforeach()

#VSCode-specific settings
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    add_custom_target(copy_compile_commands ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_SOURCE_DIR}/compile_commands.json
        COMMENT "Copying compile commands for IDE support..."
    )
endif()