# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

project(UsbTypeCTests VERSION 0.0.1
        DESCRIPTION "Tests for the UsbTypeC library"
        LANGUAGES CXX)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/tcpc.cpp
    ${CMAKE_CURRENT_LIST_DIR}/protocol_layer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/type_c.cpp
    ${CMAKE_CURRENT_LIST_DIR}/policy_engine.cpp
    ${CMAKE_CURRENT_LIST_DIR}/policy_engine_source.cpp
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_link_libraries(${PROJECT_NAME}
    usbc
)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
