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

#micro-ROS configuration
set(MICRO_ROS_DIR ${DEPS_DIR}/micro_ros_pico_sdk)
set(MICRO_ROS_PLATFORM_DIR ${MICRO_ROS_DIR}/libmicroros)

#Main project setup 
project(${PROJECT_NAME} C CXX ASM)

#Initialize Pico SDK
pico_sdk_init()
set(PICO_LWIP_COMPILER_FLAGS "-Wno-address-of-packed-member -Wno-unused-but-set-variable")

#Common sources and includes
set(COMMON_SOURCES
    ./Src/Robohand.c
    ./Include/Robohand.h
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    ${MICRO_ROS_PLATFORM_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

#Configure micro-ROS build
add_library(microros STATIC IMPORTED)
set_target_properties(microros PROPERTIES
    IMPORTED_LOCATION ${MICRO_ROS_PLATFORM_DIR}/libmicroros.a
    INTERFACE_INCLUDE_DIRECTORIES "${MICRO_ROS_PLATFORM_DIR}/include"
)

#Build configurations
set(TARGET_VARIANTS
    "_uros:pico"
    "_usb:pico"
)

foreach(TARGET_INFO IN LISTS TARGET_VARIANTS)
    string(REPLACE ":" ";" TARGET_PARTS ${TARGET_INFO})
    list(GET TARGET_PARTS 0 TARGET_SUFFIX)
    list(GET TARGET_PARTS 1 PICO_BOARD)

    set(TARGET_NAME ${PROJECT_NAME}${TARGET_SUFFIX})
    set(SOURCE_FILE ./Src/Robohand${TARGET_SUFFIX}.c)

    #Create executable with board-specific settings
    add_executable(${TARGET_NAME} 
        ${SOURCE_FILE} 
        ${COMMON_SOURCES}
    )
    
    #Set board definition for conditional compilation
    set_target_properties(${TARGET_NAME} PROPERTIES 
        PICO_BOARD ${PICO_BOARD}
    )
    
    target_compile_definitions(${TARGET_NAME} PRIVATE
        PICO_BOARD_${PICO_BOARD}
    )

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

    # micro-ROS configurations
    if(TARGET_SUFFIX MATCHES "_uros")
        target_link_libraries(${TARGET_NAME} microros)
        target_include_directories(${TARGET_NAME} PRIVATE
            ${MICRO_ROS_PLATFORM_DIR}/include
        )
    endif()

    # Common compile definitions
    target_compile_definitions(${TARGET_NAME} PRIVATE
        PICO_UART_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_DEFAULT_CRLF=0
    )

    pico_add_extra_outputs(${TARGET_NAME})
endforeach()

# VSCode-specific settings
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    add_custom_target(copy_compile_commands ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_SOURCE_DIR}/compile_commands.json
        COMMENT "Copying compile commands for IDE support..."
    )
endif()