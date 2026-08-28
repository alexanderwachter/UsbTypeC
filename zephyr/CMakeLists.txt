# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_USB_TYPEC_STACK)

zephyr_library_named(usbc)

zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/../src/UsbTypeC.cpp
)

zephyr_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_LIST_DIR}/../lib/McuTemplateLibrary/include
)

endif()
