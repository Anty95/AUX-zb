# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)

# zcl_utility (examples_utils) ships with ESP-IDF under examples/zigbee/ — portable path (Windows/Linux)
if(DEFINED ENV{IDF_PATH})
    set(EXTRA_COMPONENT_DIRS "$ENV{IDF_PATH}/examples/zigbee/zb_common_components")
endif()

# Version configuration (before project() so it's available early)
set(PROJECT_VER "0.1")
set(BUILD_NUMBER 0)
set(MANUFACTURER_CODE "0xFABC")
set(IMAGE_TYPE "0x1400")
set(ZIGBEE_STACK_VERSION "0x0003")  # Zigbee 3.0

# Convert PROJECT_VER (e.g. 1.1.0) to 0xMMmmpppp format for OTA
string(REPLACE "." ";" _ver_list ${PROJECT_VER})

list(GET _ver_list 0 _ver_major)
list(GET _ver_list 1 _ver_minor)
# Calculate OTA version dec
math(EXPR OTA_VERSION_DEC "((${_ver_major}) << 24) | ((${_ver_minor}) << 16) | (${BUILD_NUMBER})")
# Convert OTA_VERSION_DEC to 8-digit uppercase hex string (CMake <3.19 workaround)
set(_hex_chars "0123456789ABCDEF")
set(_value ${OTA_VERSION_DEC})
set(OTA_VERSION_HEX_STR "")
foreach(_i RANGE 1 8)
    math(EXPR _digit "${_value} % 16")
    string(SUBSTRING "${_hex_chars}" ${_digit} 1 _char)
    set(OTA_VERSION_HEX_STR "${_char}${OTA_VERSION_HEX_STR}")
    math(EXPR _value "${_value} / 16")
endforeach()

# Generate date code
string(TIMESTAMP DATE_CODE "%Y%m%d")

# Generate version header early (before project() call)
# Create directory and generate header so components can include it
if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/generated")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
endif()
configure_file(${CMAKE_SOURCE_DIR}/version.h.in ${CMAKE_CURRENT_BINARY_DIR}/generated/version.h @ONLY)

project(h-link_zb)

# (Top-level compile defs removed - version header is generated and used by components)
# Find Python
find_package(Python3 REQUIRED)

# --- Optional Zigbee OTA .ota file (Espressif image_builder_tool from esp-zigbee-sdk) ---
# Default idf.py build does NOT run this (no ALL): flash the normal .bin.
# Clone https://github.com/espressif/esp-zigbee-sdk and either:
#   - set env ESP_ZIGBEE_SDK to the repo root, or
#   - cmake -DZIGBEE_OTA_IMAGE_BUILDER=C:/path/to/esp-zigbee-sdk/tools/image_builder_tool/image_builder_tool.py
set(ZIGBEE_OTA_IMAGE_BUILDER "" CACHE FILEPATH "Path to image_builder_tool.py (empty = skip generate_ota target)")
set(_ota_script "${ZIGBEE_OTA_IMAGE_BUILDER}")
if(_ota_script STREQUAL "" AND DEFINED ENV{ESP_ZIGBEE_SDK})
    set(_ota_script "$ENV{ESP_ZIGBEE_SDK}/tools/image_builder_tool/image_builder_tool.py")
    file(TO_CMAKE_PATH "${_ota_script}" _ota_script)
endif()

if(EXISTS "${_ota_script}")
    add_custom_target(generate_ota
        COMMAND ${CMAKE_COMMAND} -E echo "================================================"
        COMMAND ${CMAKE_COMMAND} -E echo "Generating Zigbee OTA image:"
        COMMAND ${CMAKE_COMMAND} -E echo "  Version: ${PROJECT_VER}.${BUILD_NUMBER} - 0x${OTA_VERSION_HEX_STR} - ${OTA_VERSION_DEC}"
        COMMAND ${CMAKE_COMMAND} -E echo "  Manufacturer: ${MANUFACTURER_CODE}"
        COMMAND ${CMAKE_COMMAND} -E echo "  Image Type: ${IMAGE_TYPE}"
        COMMAND ${CMAKE_COMMAND} -E echo "  Stack Version: ${ZIGBEE_STACK_VERSION}"
        COMMAND ${CMAKE_COMMAND} -E echo "================================================"
        COMMAND ${Python3_EXECUTABLE}
            "${_ota_script}"
            -f "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin"
            -c "${CMAKE_BINARY_DIR}/${PROJECT_NAME}_v${PROJECT_VER}.${BUILD_NUMBER}.ota"
            -m ${MANUFACTURER_CODE}
            -i ${IMAGE_TYPE}
            -v 0x${OTA_VERSION_HEX_STR}
            -s ${ZIGBEE_STACK_VERSION}
        COMMAND ${CMAKE_COMMAND} -E echo "OTA file generated: ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_v${PROJECT_VER}.${BUILD_NUMBER}.ota"
        DEPENDS ${PROJECT_NAME}.elf
        BYPRODUCTS "${CMAKE_BINARY_DIR}/${PROJECT_NAME}_v${PROJECT_VER}.${BUILD_NUMBER}.ota"
        COMMENT "Generating Zigbee OTA image v${PROJECT_VER} (0x${OTA_VERSION_HEX_STR})"
    )
    message(STATUS "Zigbee OTA: optional target 'generate_ota' (image_builder: ${_ota_script})")
else()
    message(STATUS "Zigbee OTA: image_builder_tool.py not found — set ZIGBEE_OTA_IMAGE_BUILDER or ESP_ZIGBEE_SDK to enable target generate_ota")
endif()

