# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_DRV2605_ALT src/drv2605_alt.c)
zephyr_library_sources_ifdef(CONFIG_DRV2605_ALT_GPIO src/drv2605_alt_gpio.c)
zephyr_include_directories(include)

zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
