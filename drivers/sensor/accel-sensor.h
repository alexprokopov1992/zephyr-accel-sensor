#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>

enum accel_sensor_mode {
    ACCEL_SENSOR_MODE_ARMED=0,
    ACCEL_SENSOR_MODE_DISARMED,
    ACCEL_SENSOR_MODE_TURN_OFF,
    ACCEL_SENSOR_MODE_ALARM,
    ACCEL_SENSOR_MODE_ALARM_STOP,
};

enum accel_sensor_attrs {
    ACCEL_SENSOR_SPECIAL_ATTRS=64,
};

enum accel_sensor_channel {
    ACCEL_SENSOR_MODE=128,
    ACCEL_SENSOR_CHANNEL_WARN_ZONE,
    ACCEL_SENSOR_CHANNEL_MAIN_ZONE,
    ACCEL_SENSOR_INCREASE_SENSIVITY_INTERVAL_SEC,
};

struct accel_sensor_config {
	// const struct i2c_dt_spec bus;
	const struct device *accel_dev;
};

typedef struct {
	float x, y, z;
} _Vector3;

struct accel_sensor_data {
	uint16_t sampling_period_ms;
	struct k_work_delayable dwork;
	const struct device *accel_dev;
	sensor_trigger_handler_t warn_handler;
    const struct sensor_trigger *warn_trigger;
    sensor_trigger_handler_t main_handler;
    const struct sensor_trigger *main_trigger;
	_Vector3 ref_acc;		// Еталонне положення пристрою
	_Vector3 last_acc;		// Останнє виміряне прискорення
	float main_zone_steps[10];
	float main_zone_cos_pow2[10];
	float warn_zone_cos_pow2[10];
	int selected_warn_zone;
	int current_warn_zone;
	int selected_main_zone;
	int current_main_zone;
	int mode;
	bool in_warn_alert;
	bool in_main_alert;
	struct k_timer refresh_current_pos_timer;
	struct k_timer increase_sensivity_timer;
};


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

#if 0
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

typedef int (*set_current_position_as_reference_t)(const struct device *dev);
typedef int (*set_sensor_settings_t)(const struct device *dev, int channel, int val1, int val2);

__subsystem struct accel_sensor_driver_api {
	set_current_position_as_reference_t set_current_position_as_reference;
	set_sensor_settings_t attr_set;
	// sensor_attr_get_t attr_get;
	// sensor_trigger_set_t trigger_set;
	// sensor_sample_fetch_t sample_fetch;
	// sensor_channel_get_t channel_get;
	// sensor_get_decoder_t get_decoder;
	// sensor_submit_t submit;
};

static inline int accel_sensor_attr_set(const struct device *dev, int channel, int val1, int val2)
{
	const struct accel_sensor_driver_api *api = (const struct accel_sensor_driver_api *)dev->api;

	if (api->attr_set == NULL) {
		return -ENOSYS;
	}

	return api->attr_set(dev, channel, val1, val2);
}


static inline int accel_sensor_set_current_position_as_reference(const struct device *dev)
{
	const struct accel_sensor_driver_api *api = (const struct accel_sensor_driver_api *)dev->api;

	if (api->set_current_position_as_reference == NULL) {
		return -ENOSYS;
	}

	return api->set_current_position_as_reference(dev);
}