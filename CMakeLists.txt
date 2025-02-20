# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_definitions(traceTASK_SWITCHED_OUT=log_event_on_exit;traceTASK_SWITCHED_IN=log_event_on_enter)
add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wno-implicit-function-declaration>)
project(usb-audio)
