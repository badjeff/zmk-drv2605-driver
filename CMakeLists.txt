# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_DRV2605 src/drv2605.c)
zephyr_include_directories(include)

zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
