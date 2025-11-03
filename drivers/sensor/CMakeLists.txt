# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LIS2DW12_ACCEL_SENSOR 
    lis2dw12-sensor.c
    lis2dw12-sensor-init.c
    lis2dw12-sensor-logic.c
)
zephyr_library_sources_ifdef(CONFIG_LIS2DW12_ACCEL_SENSOR lis2dw12-sensor.c)
