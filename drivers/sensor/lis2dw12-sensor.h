#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

static float warn_zone_start_angle = 1.0;
static float warn_zone_step_angle = 2.0/9.0;
static float max_angle = 10.00;
static const float cos_pow_0_5  = 0.999961923;

static float border_move = 0.0005;

static float warn_zone_accel_mult = 0.001;
static float warn_zone_step_accel_mult_step = 0.001;
static float main_zone_max_mult = 0.1;

#if !defined(M_PIf)
#define M_PIf 3.1415927f
#endif

#define MOVE_SENSOR_SAMPLE_TIME 20
#define MOVE_SENSOR_SAMPLE_COUNT 5
#define ACCEL_SENSOR_SAMPLE_TIME 1000

#define REFRESH_POS_TIME 3600
#define REFRESH_POS_TIME_MOVE 10
#define INCREASE_SENSIVITY_TIME 10
#define ARMING_DELAY_SEC 10
#define ARMING_DELAY_SEC_DIS 1
#define MIN_WARN_INTERVAL 2000 // ms
#define STOP_ACCEL_ALARM_INTERVAL 5000
/* LIS2DW12 Register addresses */
#define LIS2DW12_REG_OUT_T_L        0x0D
#define LIS2DW12_REG_OUT_T_H        0x0E
#define LIS2DW12_REG_WHO_AM_I       0x0F
#define LIS2DW12_REG_CTRL1          0x20
#define LIS2DW12_REG_CTRL2          0x21
#define LIS2DW12_REG_CTRL3          0x22
#define LIS2DW12_REG_CTRL4_INT1     0x23
#define LIS2DW12_REG_CTRL5_INT2     0x24
#define LIS2DW12_REG_CTRL6          0x25
#define LIS2DW12_REG_OUT_T          0x26
#define LIS2DW12_REG_STATUS         0x27
#define LIS2DW12_REG_OUT_X_L        0x28
#define LIS2DW12_REG_OUT_X_H        0x29
#define LIS2DW12_REG_OUT_Y_L        0x2A
#define LIS2DW12_REG_OUT_Y_H        0x2B
#define LIS2DW12_REG_OUT_Z_L        0x2C
#define LIS2DW12_REG_OUT_Z_H        0x2D
#define LIS2DW12_REG_FIFO_CTRL      0x2E
#define LIS2DW12_REG_FIFO_SAMPLES   0x2F
#define LIS2DW12_REG_TAP_THS_X      0x30
#define LIS2DW12_REG_TAP_THS_Y      0x31
#define LIS2DW12_REG_TAP_THS_Z      0x32
#define LIS2DW12_REG_INT_DUR        0x33
#define LIS2DW12_REG_WAKE_UP_THS    0x34
#define LIS2DW12_REG_WAKE_UP_DUR    0x35
#define LIS2DW12_REG_FREE_FALL      0x36
#define LIS2DW12_REG_STATUS_DUP     0x37
#define LIS2DW12_REG_WAKE_UP_SRC    0x38
#define LIS2DW12_REG_TAP_SRC        0x39
#define LIS2DW12_REG_SIXD_SRC       0x3A
#define LIS2DW12_REG_ALL_INT_SRC    0x3B
#define LIS2DW12_REG_X_OFS_USR      0x3C
#define LIS2DW12_REG_Y_OFS_USR      0x3D
#define LIS2DW12_REG_Z_OFS_USR      0x3E
#define LIS2DW12_REG_CTRL7          0x3F

/* WHO_AM_I value */
#define LIS2DW12_WHO_AM_I_VALUE     0x44

/* CTRL1 register bits */
#define LIS2DW12_CTRL1_ODR_SHIFT    4
#define LIS2DW12_CTRL1_ODR_MASK     0xF0
#define LIS2DW12_CTRL1_MODE_SHIFT   2
#define LIS2DW12_CTRL1_MODE_MASK    0x0C
#define LIS2DW12_CTRL1_LP_MODE_SHIFT 0
#define LIS2DW12_CTRL1_LP_MODE_MASK 0x03

/* CTRL2 register bits */
#define LIS2DW12_CTRL2_BOOT         0x80
#define LIS2DW12_CTRL2_SOFT_RESET   0x40
#define LIS2DW12_CTRL2_CS_PU_DISC   0x10
#define LIS2DW12_CTRL2_BDU          0x08
#define LIS2DW12_CTRL2_IF_ADD_INC   0x04

/* CTRL3 register bits */
#define LIS2DW12_CTRL3_SLP_MODE_1   0x02
#define LIS2DW12_CTRL3_SLP_MODE_SEL 0x01

/* CTRL4_INT1_PAD_CTRL register bits */
#define LIS2DW12_CTRL4_INT1_DRDY    0x01

/* CTRL6 register bits */
#define LIS2DW12_CTRL6_BW_FILT_SHIFT 6
#define LIS2DW12_CTRL6_BW_FILT_MASK 0xC0
#define LIS2DW12_CTRL6_FS_SHIFT     4
#define LIS2DW12_CTRL6_FS_MASK      0x30
#define LIS2DW12_CTRL6_FDS          0x08
#define LIS2DW12_CTRL6_LOW_NOISE    0x04

/* STATUS register bits */
#define LIS2DW12_STATUS_DRDY        0x01

/* SPI read/write bit */
#define LIS2DW12_SPI_READ           0x80
#define LIS2DW12_SPI_WRITE          0x00
#define LIS2DW12_SPI_AUTO_INC       0x40

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
	ACCEL_DISARM_TRIGGER_MOVE,
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

typedef struct {
	float x, y, z;
} _Vector3;

struct lis2dw12_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec int_gpio;
};

struct lis2dw12_data {
	uint16_t sampling_period_ms;
	struct k_work_delayable dwork;
	
	/* Interrupt handling */
	struct gpio_callback gpio_cb;
	const struct device *dev;
	struct k_sem data_ready_sem;
	
	/* Raw accelerometer data */
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	
	/* Tilt detection fields */
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
	
	/* Movement detection fields */
	sensor_trigger_handler_t warn_handler_move;
    const struct sensor_trigger *warn_trigger_move;
    sensor_trigger_handler_t main_handler_move;
    const struct sensor_trigger *main_trigger_move;

	sensor_trigger_handler_t disarm_move_handler;
	const struct sensor_trigger *disarm_move_trigger;
	
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

    _Vector3 last_acc_move;
	_Vector3 ref_acc_move;
	int samples_count_move;
	_Vector3 summary_acc_move;
	float gravity;
	struct k_timer refresh_current_pos_timer_move;

	_Vector3 last_acc_move_disarmed;
	_Vector3 ref_acc_move_disarmed;
	int samples_count_move_disarmed;
	_Vector3 summary_acc_move_disarmed;
	float gravity_disarmed;
	struct k_timer refresh_current_pos_timer_move_disarmed;
	int64_t last_trigger_time_disarmed_move;

	int64_t last_trigger_time_warn_move;
    int64_t last_trigger_time_main_move;
	struct k_timer increase_sensivity_warn_timer_move;
	struct k_timer increase_sensivity_main_timer_move;
	struct k_timer alarm_timer_move;
};

typedef int (*sensor_attr_set_t)(const struct device *dev,
	enum sensor_channel chan,
	enum sensor_attribute attr,
	const struct sensor_value *val);

typedef int (*set_current_position_as_reference_t)(const struct device *dev);
typedef int (*set_sensor_settings_t)(const struct device *dev, int channel, int val1, int val2);
typedef int (*sensor_trigger_set_t)(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler);

__subsystem struct accel_sensor_driver_api {
	set_current_position_as_reference_t set_current_position_as_reference;
	set_sensor_settings_t attr_set;
	sensor_trigger_set_t trigger_set;
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
