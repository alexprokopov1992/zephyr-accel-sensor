/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "drivers/sensor/lis2dw12-sensor.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Get accelerometer device from device tree alias */
const struct device *accel = DEVICE_DT_GET(DT_ALIAS(accel0));

/* Callback for tilt warning */
static void tilt_warn_callback(const struct device *dev, 
                                const struct sensor_trigger *trig)
{
	LOG_WRN("⚠️  TILT WARNING detected!");
}

/* Callback for tilt alarm */
static void tilt_alarm_callback(const struct device *dev, 
                                 const struct sensor_trigger *trig)
{
	LOG_ERR("🚨 TILT ALARM! Critical angle exceeded!");
}

/* Callback for movement warning */
static void move_warn_callback(const struct device *dev, 
                                const struct sensor_trigger *trig)
{
	LOG_WRN("⚠️  MOVEMENT WARNING detected!");
}

/* Callback for movement alarm */
static void move_alarm_callback(const struct device *dev, 
                                 const struct sensor_trigger *trig)
{
	LOG_ERR("🚨 MOVEMENT ALARM! Significant displacement!");
}

/* Callback for disarm event */
static void disarm_callback(const struct device *dev, 
                             const struct sensor_trigger *trig)
{
	LOG_INF("✅ System armed and ready for monitoring");
}

void main(void)
{
	int ret;

	LOG_INF("LIS2DW12 Accelerometer Example");
	LOG_INF("==============================");

	/* Check if device is ready */
	if (!device_is_ready(accel)) {
		LOG_ERR("Accelerometer device not ready");
		return;
	}

	LOG_INF("✓ Accelerometer device ready");

	/* Configure tilt warning trigger */
	struct sensor_trigger tilt_warn_trig = {
		.type = ACCEL_WARN_TRIGGER,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	ret = accel_sensor_trigger_set(accel, &tilt_warn_trig, tilt_warn_callback);
	if (ret < 0) {
		LOG_ERR("Failed to set tilt warning trigger");
		return;
	}

	/* Configure tilt alarm trigger */
	struct sensor_trigger tilt_alarm_trig = {
		.type = ACCEL_MAIN_TRIGGER,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	ret = accel_sensor_trigger_set(accel, &tilt_alarm_trig, tilt_alarm_callback);
	if (ret < 0) {
		LOG_ERR("Failed to set tilt alarm trigger");
		return;
	}

	/* Configure movement warning trigger */
	struct sensor_trigger move_warn_trig = {
		.type = ACCEL_WARN_TRIGGER_MOVE,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	ret = accel_sensor_trigger_set(accel, &move_warn_trig, move_warn_callback);
	if (ret < 0) {
		LOG_ERR("Failed to set movement warning trigger");
		return;
	}

	/* Configure movement alarm trigger */
	struct sensor_trigger move_alarm_trig = {
		.type = ACCEL_MAIN_TRIGGER_MOVE,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	ret = accel_sensor_trigger_set(accel, &move_alarm_trig, move_alarm_callback);
	if (ret < 0) {
		LOG_ERR("Failed to set movement alarm trigger");
		return;
	}

	/* Configure disarm callback */
	struct sensor_trigger disarm_trig = {
		.type = ACCEL_DISARM_TRIGGER_MOVE,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	ret = accel_sensor_trigger_set(accel, &disarm_trig, disarm_callback);
	if (ret < 0) {
		LOG_ERR("Failed to set disarm trigger");
		return;
	}

	LOG_INF("✓ All triggers configured");

	/* Configure tilt detection zones */
	/* Warning zone level 5 (medium sensitivity) */
	ret = accel_sensor_attr_set(accel, ACCEL_SENSOR_CHANNEL_WARN_ZONE, 50, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set warning zone");
		return;
	}

	/* Alarm zone level 5 (medium sensitivity) */
	ret = accel_sensor_attr_set(accel, ACCEL_SENSOR_CHANNEL_MAIN_ZONE, 50, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set main zone");
		return;
	}

	/* Configure movement detection zones */
	ret = accel_sensor_attr_set(accel, ACCEL_SENSOR_CHANNEL_WARN_ZONE_MOVE, 50, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set movement warning zone");
		return;
	}

	ret = accel_sensor_attr_set(accel, ACCEL_SENSOR_CHANNEL_MAIN_ZONE_MOVE, 50, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set movement main zone");
		return;
	}

	LOG_INF("✓ Detection zones configured");

	/* Wait for device to stabilize */
	LOG_INF("Waiting for sensor to stabilize...");
	k_sleep(K_SECONDS(2));

	/* Save current position as reference */
	ret = accel_sensor_set_current_position_as_reference(accel);
	if (ret < 0) {
		LOG_ERR("Failed to set reference position");
		return;
	}

	LOG_INF("✓ Reference position set");

	/* Enable tilt monitoring */
	ret = accel_sensor_attr_set(accel, ACCEL_SENSOR_MODE, 
	                             ACCEL_SENSOR_MODE_ARMED, 0);
	if (ret < 0) {
		LOG_ERR("Failed to arm tilt monitoring");
		return;
	}

	/* Enable movement monitoring */
	ret = accel_sensor_attr_set(accel, ACCEL_SENSOR_MODE_MOVE, 
	                             ACCEL_SENSOR_MODE_ARMED, 0);
	if (ret < 0) {
		LOG_ERR("Failed to arm movement monitoring");
		return;
	}

	LOG_INF("✓ System armed - monitoring started");
	LOG_INF("");
	LOG_INF("System is now monitoring for:");
	LOG_INF("  - Tilt changes (angle deviation)");
	LOG_INF("  - Movement (acceleration changes)");
	LOG_INF("");
	LOG_INF("Try tilting or moving the device...");

	/* Main loop - just keep running */
	while (1) {
		k_sleep(K_FOREVER);
	}
}
