# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

project("UsbTypeC" VERSION 0.0.1
        DESCRIPTION "USB Type-C state machines with Power Delivery"
        LANGUAGES CXX)

set(TARGET_NAME usbc)

option(ENABLE_TESTING "Enable the tests" ${PROJECT_IS_TOP_LEVEL})

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/McuTemplateLibrary)

add_library(${TARGET_NAME} STATIC
    ${CMAKE_CURRENT_LIST_DIR}/src/UsbTypeC.cpp
)
target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)

target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(${TARGET_NAME} PUBLIC mtl)

if(ENABLE_TESTING)
    enable_testing()
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test)
endif()
