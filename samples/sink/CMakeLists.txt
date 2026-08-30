# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usbc_sink_sample LANGUAGES CXX)

target_sources(app PRIVATE src/main.cpp)
