# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

project(UsbTypeCTools VERSION 0.0.1
        DESCRIPTION "Development tools for the UsbTypeC library"
        LANGUAGES CXX)

add_executable(usbc-dotgen
    ${CMAKE_CURRENT_LIST_DIR}/dotgen.cpp
)

target_compile_features(usbc-dotgen PRIVATE cxx_std_23)
target_link_libraries(usbc-dotgen
    usbc
)

# "cmake --build <build dir> --target dot" renders every state machine
add_custom_target(dot
    COMMAND usbc-dotgen ${CMAKE_BINARY_DIR}/dot
    COMMENT "Generating state machine DOT files in ${CMAKE_BINARY_DIR}/dot"
)
