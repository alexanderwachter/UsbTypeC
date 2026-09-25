# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_USB_TYPEC_STACK)

# mtl comes in as a west module of its own (west.yml): its include
# paths, the fsm debug-check policy (MTL_FSM_CHECKS follows
# CONFIG_ASSERT), and the trace logger need no wiring here

zephyr_library_named(usbc)

zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/../src/UsbTypeC.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Tcpc.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/WorkQueue.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Vbus.cpp
)

zephyr_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_LIST_DIR}/include
)

endif()
