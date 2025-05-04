#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define MMA8652_ADDR 0x1D
#define CTRL_REG1      0x2A
#define FF_MT_CFG      0x15
#define FF_MT_THS      0x17
#define FF_MT_COUNT    0x18
#define CTRL_REG4      0x2D
#define CTRL_REG5      0x2E
#define F_SETUP        0x09


enum accel_sensor_mode {
    ACCEL_SENSOR_MODE_ARMED=0,
    ACCEL_SENSOR_MODE_DISARMED,
    ACCEL_SENSOR_MODE_ALARM,
    ACCEL_SENSOR_MODE_ALARM_STOP,
};

enum accel_sensor_attrs {
    ACCEL_SENSOR_SPECIAL_ATTRS=64,
	ACCEL_SENSOR_CHAN_XYZ,
};

enum accel_sensor_trigger_types {
	ACCEL_WARN_TRIGGER,
	ACCEL_MAIN_TRIGGER,
	ACCEL_WARN_TRIGGER_MOVE,
	ACCEL_MAIN_TRIGGER_MOVE,
};

enum accel_sensor_channel {
    ACCEL_SENSOR_MODE=128,
    ACCEL_SENSOR_CHANNEL_WARN_ZONE,
    ACCEL_SENSOR_CHANNEL_MAIN_ZONE,
    ACCEL_SENSOR_INCREASE_SENSIVITY_INTERVAL_SEC,
	ACCEL_SENSOR_MODE_MOVE,
	ACCEL_SENSOR_CHANNEL_WARN_ZONE_MOVE,
	ACCEL_SENSOR_CHANNEL_MAIN_ZONE_MOVE,
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
	//поля стуртури для нахилу
	sensor_trigger_handler_t warn_handler_tilt;
    const struct sensor_trigger *warn_trigger_tilt;
    sensor_trigger_handler_t main_handler_tilt;
    const struct sensor_trigger *main_trigger_tilt;
	_Vector3 ref_acc_tilt;
	_Vector3 last_acc_tilt;
	float main_zone_cos_pow2[10];
	float warn_zone_cos_pow2[10];
	int selected_warn_zone_tilt;
	int current_warn_zone_tilt;
	int selected_main_zone_tilt;
	int current_main_zone_tilt;
	int mode_tilt;
	bool in_warn_alert_tilt;
	bool in_main_alert_tilt;
	bool max_warn_alert_level_tilt;
	bool max_main_alert_level_tilt;
	bool warn_zone_active_tilt;
	bool main_zone_active_tilt;

	int64_t last_trigger_time_warn_tilt;
    int64_t last_trigger_time_main_tilt;

	struct k_timer refresh_current_pos_timer_tilt;
	struct k_timer increase_sensivity_timer_tilt;
	struct k_timer alarm_timer_tilt;

	int skip_counter;
	//поля структури для переміщення
	sensor_trigger_handler_t warn_handler_move;
    const struct sensor_trigger *warn_trigger_move;
    sensor_trigger_handler_t main_handler_move;
    const struct sensor_trigger *main_trigger_move;
	_Vector3 ref_acc_move;
	_Vector3 last_acc_move;
	float main_zone_move[10];
	float warn_zone_move[10];
	int selected_warn_zone_move;
	int current_warn_zone_move;
	int selected_main_zone_move;
	int current_main_zone_move;
	int mode_move;
	bool max_warn_alert_level_move;
	bool max_main_alert_level_move;
	bool warn_zone_active_move;
	bool main_zone_active_move;

    int samples_count_move;
	_Vector3 summary_acc_move;

	float gravity;

	int64_t last_trigger_time_warn_move;
    int64_t last_trigger_time_main_move;
	struct k_timer refresh_current_pos_timer_move;
	struct k_timer increase_sensivity_warn_timer_move;
	struct k_timer increase_sensivity_main_timer_move;
	struct k_timer alarm_timer_move;
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
typedef int (*sensor_trigger_set_t)(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler);

__subsystem struct accel_sensor_driver_api {
	set_current_position_as_reference_t set_current_position_as_reference;
	set_sensor_settings_t attr_set;
	sensor_trigger_set_t trigger_set;
	// sensor_attr_get_t attr_get;
	// sensor_sample_fetch_t sample_fetch;
	// sensor_channel_get_t channel_get;
	// sensor_get_decoder_t get_decoder;
	// sensor_submit_t submit;
};


static inline int accel_sensor_trigger_set(const struct device *dev, const struct sensor_trigger *trig,	sensor_trigger_handler_t handler)
{
	const struct accel_sensor_driver_api *api = (const struct accel_sensor_driver_api *)dev->api;

	if (api->trigger_set == NULL) {
		return -ENOSYS;
	}

	return api->trigger_set(dev, trig, handler);
}

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