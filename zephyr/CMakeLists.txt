# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_USB_TYPEC_STACK)

# mtl comes in as a west module of its own (west.yml): its include
# paths, the fsm debug-check policy (MTL_FSM_CHECKS follows
# CONFIG_ASSERT), and the trace logger need no wiring here

zephyr_library_named(usbc)

zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/src/Tcpc.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/WorkQueue.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Vbus.cpp
)

zephyr_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# "west build -t dot" draws the state machines of the image just built,
# for mtl's fsmview: the tables come from the ELF's symbols, every
# instantiation as the image runs it, the graphs from the stack's and
# the application's headers through mtl's generator
add_custom_target(dot
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_MTL_MODULE_DIR}/tools/dotgen/dotgen.py
            ${CMAKE_CURRENT_LIST_DIR}/../include ${APPLICATION_SOURCE_DIR}
            --elf ${PROJECT_BINARY_DIR}/${KERNEL_ELF_NAME} --nm ${CMAKE_NM}
            -o ${CMAKE_BINARY_DIR}/dot
    DEPENDS ${ZEPHYR_FINAL_EXECUTABLE}
    COMMENT "Writing the graphs of the image's state machines into ${CMAKE_BINARY_DIR}/dot"
    VERBATIM
)

endif()

# Drivers the stack's boards need, independent of the stack itself
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/drivers)
