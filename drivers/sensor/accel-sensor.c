#define DT_DRV_COMPAT baden_accel_sensor

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

#include "accel-sensor.h"

LOG_MODULE_REGISTER(accel_sensor, LOG_LEVEL_DBG);

static int init(const struct device *dev)
{
	LOG_DBG("Initializing Accelerometer Sensor");
	const struct accel_sensor_config *cfg = dev->config;
	struct accel_sensor_data *data = dev->data;
	int rc;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("Failed to get pointer to %s device!", cfg->bus.bus->name);
		return -ENODEV;
	}

    #if 0
	rc = _set_sample_period(dev, data->sample_period_ms);
	if (rc != 0) {
		LOG_WRN("Failed to set sample period. Using period stored of device");
		/* Try to read sample time from sensor to reflect actual sample period */
		rc = _get_sample_time(dev);
	}

	LOG_DBG("Sample period: %d", data->sample_period_ms);
	if (rc != 0) {
		return rc;
	}
    #endif
    
	LOG_DBG("Starting periodic measurements");
	rc = 0; /* scd30_write_register(dev, SCD30_CMD_START_PERIODIC_MEASUREMENT,
				  SCD30_MEASUREMENT_DEF_AMBIENT_PRESSURE);*/

	return rc;
}

#define ACCEL_SENSOR_DEFINE(inst)									    \
	static struct accel_sensor_data data_##inst = {						\
		.sample_period_ms = DT_INST_PROP(inst, sample_period_ms)		\
	};											                        \
	static const struct accel_sensor_config config_##inst = {			\
		.bus = I2C_DT_SPEC_INST_GET(inst),						        \
	};											                        \
												                        \
	DEVICE_DT_INST_DEFINE(inst, init, NULL, &data_##inst, &config_##inst,	\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &scd30_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ACCEL_SENSOR_DEFINE);
