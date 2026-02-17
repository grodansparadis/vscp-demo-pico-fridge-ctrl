# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(w55rp20_fridge_ctrl C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(ARCHIVE_OUTPUT_DIRECTORY ./build)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# WIZnet libraries
set(BOARD_NAME W55RP20_EVB_PICO)
set(WIZNET_CHIP W5500)
add_definitions(-D_WIZCHIP_=W5500)
add_definitions(-DDEVICE_BOARD_NAME=W55RP20_EVB_PICO)
message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")
include(${CMAKE_SOURCE_DIR}/WIZnet-PICO-C/wiznet_pico_c_sdk_version.cmake)

set(TARGET_NAME w55rp20_fridge_ctrl)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        lcd_1602_i2c.c
        ${CMAKE_SOURCE_DIR}/vscp-firmware/common/vscp-firmware-helper.c
        ${CMAKE_SOURCE_DIR}/vscp-firmware/common/vscp-firmware-level2.c
        vscp-firmware/third-party/cJSON/cJSON.c
        )

target_include_directories(${TARGET_NAME} PRIVATE
        .
        WIZnet-PICO-C/port/
        WIZnet-PICO-C/port/ioLibrary_Driver/inc/
        WIZnet-PICO-C/libraries/ioLibrary_Driver/Ethernet/
        WIZnet-PICO-C/libraries/ioLibrary_Driver/Internet/MQTT/
        vscp-firmware/common 
        vscp-firmware/third-party/cJSON
        vscp/src/vscp/common
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_adc
        ETHERNET_FILES
        IOLIBRARY_FILES
        MQTT_FILES
        DHCP_FILES
        DNS_FILES
        TIMER_FILES
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(w55rp20_fridge_ctrl)
pico_add_uf2_output(w55rp20_fridge_ctrl)

# Set the project root directory if it's not already defined, as may happen if
# the tests folder is included directly by a parent project, without including
# the top level CMakeLists.txt.
if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/WIZnet-PICO-C/libraries/ioLibrary_Driver)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()

if(NOT DEFINED MBEDTLS_DIR)
    set(MBEDTLS_DIR ${CMAKE_SOURCE_DIR}/WIZnet-PICO-C/libraries/mbedtls)
    message(STATUS "MBEDTLS_DIR = ${MBEDTLS_DIR}")
endif()

if(NOT DEFINED PORT_DIR)
    set(PORT_DIR ${CMAKE_SOURCE_DIR}/WIZnet-PICO-C/port)
    message(STATUS "PORT_DIR = ${PORT_DIR}")
endif()

# Turn off mbedtls test mode 
set(ENABLE_PROGRAMS OFF CACHE BOOL "Build mbedtls programs")
set(ENABLE_TESTING OFF CACHE BOOL "Build mbedtls testing")
add_definitions(-DMBEDTLS_CONFIG_FILE="${PORT_DIR}/mbedtls/inc/ssl_config.h")
add_definitions(-DSET_TRUSTED_CERT_IN_SAMPLES)

# Add libraries in subdirectories
add_subdirectory(${CMAKE_SOURCE_DIR}/WIZnet-PICO-C/libraries ./build/lib)
#add_subdirectory(${MBEDTLS_DIR} ./build/lib-mdbedtls)
add_subdirectory(${PORT_DIR} ./build/lib-port)

# Set compile options
add_compile_options(
        -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )