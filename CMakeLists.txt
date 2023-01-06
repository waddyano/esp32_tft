# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(CMAKE_COLOR_DIAGNOSTICS OFF)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tft)
littlefs_create_partition_image(littlefs flash_data)
