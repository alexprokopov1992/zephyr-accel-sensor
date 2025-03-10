#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>

struct accel_sensor_config {
	// const struct i2c_dt_spec bus;
	const struct device *accel_dev;
};

struct accel_sensor_data {
	uint16_t sampling_period_ms;
};

#if 0
/**
 * @typedef sensor_attr_set_t
 * @brief Callback API upon setting a sensor's attributes
 *
 * See sensor_attr_set() for argument description
 */
typedef int (*sensor_attr_set_t)(const struct device *dev,
	enum sensor_channel chan,
	enum sensor_attribute attr,
	const struct sensor_value *val);

/**
 * @typedef sensor_attr_get_t
 * @brief Callback API upon getting a sensor's attributes
 *
 * See sensor_attr_get() for argument description
 */
typedef int (*sensor_attr_get_t)(const struct device *dev,
	enum sensor_channel chan,
	enum sensor_attribute attr,
	struct sensor_value *val);

#endif

__subsystem struct accel_sensor_driver_api {
	// sensor_attr_set_t attr_set;
	// sensor_attr_get_t attr_get;
	// sensor_trigger_set_t trigger_set;
	// sensor_sample_fetch_t sample_fetch;
	// sensor_channel_get_t channel_get;
	// sensor_get_decoder_t get_decoder;
	// sensor_submit_t submit;
};
