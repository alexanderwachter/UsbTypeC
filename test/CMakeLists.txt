# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

project(UsbTypeCTests VERSION 0.0.1
        DESCRIPTION "Tests for the UsbTypeC library"
        LANGUAGES CXX)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/compliance.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tcpc.cpp
    ${CMAKE_CURRENT_LIST_DIR}/protocol_layer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/type_c.cpp
    ${CMAKE_CURRENT_LIST_DIR}/type_c_drp.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pd_drp.cpp
    ${CMAKE_CURRENT_LIST_DIR}/policy_engine.cpp
    ${CMAKE_CURRENT_LIST_DIR}/policy_engine_source.cpp
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)

# -O1 compiles the template-heavy TUs measurably faster than -O0: the
# backend otherwise churns through tens of thousands of tiny
# uninlined instantiations (pd_drp.cpp: 33 s -> 20 s)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:>:-O1>)

# The stack never uses RTTI (embedded target; polymorphism is
# compile-time); -fno-rtti makes a violation fail loudly
target_compile_options(${PROJECT_NAME} PRIVATE -fno-rtti)
target_link_libraries(${PROJECT_NAME}
    usbc
)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
