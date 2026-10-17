# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(usbc_ppc_tcpp03_m20)
zephyr_library_sources(st_tcpp03_m20.c)

# the driver's public header, zephyr/drivers/usb_c/tcpp03_m20.h
zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/../../../include)
