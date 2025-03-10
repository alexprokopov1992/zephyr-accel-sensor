#pragma once


struct accel_sensor_config {
	const struct i2c_dt_spec bus;
};

struct accel_sensor_data {
	uint16_t sample_period_ms;
};
