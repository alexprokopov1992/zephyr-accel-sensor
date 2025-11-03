#define DT_DRV_COMPAT fenix_lis2dw12_accel

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <math.h>

#include "lis2dw12-sensor.h"

LOG_MODULE_REGISTER(lis2dw12_sensor, CONFIG_SENSOR_LOG_LEVEL);


/* SPI read/write functions */
static int lis2dw12_spi_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct lis2dw12_config *cfg = dev->config;
	uint8_t buffer_tx[2] = { reg | LIS2DW12_SPI_READ | LIS2DW12_SPI_AUTO_INC, 0 };
	const struct spi_buf tx_buf = {
		.buf = buffer_tx,
		.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int lis2dw12_spi_write_reg(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct lis2dw12_config *cfg = dev->config;
	uint8_t buffer_tx[16];
	
	if (len > 15) {
		return -EINVAL;
	}

	buffer_tx[0] = reg | LIS2DW12_SPI_WRITE | LIS2DW12_SPI_AUTO_INC;
	memcpy(&buffer_tx[1], data, len);

	const struct spi_buf tx_buf = {
		.buf = buffer_tx,
		.len = len + 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	return spi_write_dt(&cfg->spi, &tx);
}

static int lis2dw12_write_byte(const struct device *dev, uint8_t reg, uint8_t value)
{
	return lis2dw12_spi_write_reg(dev, reg, &value, 1);
}

static int lis2dw12_read_byte(const struct device *dev, uint8_t reg, uint8_t *value)
{
	return lis2dw12_spi_read_reg(dev, reg, value, 1);
}

/* Vector math functions */
float vector_length(_Vector3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

float vector_length_pow2(_Vector3 v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

float angle_between_vectors(_Vector3 v1, _Vector3 v2) {
    float dot_product = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    float magnitude_product = vector_length(v1) * vector_length(v2);
    if (magnitude_product == 0) return 0.0f;
    return acosf(dot_product / magnitude_product);
}

float cospow2_between_vectors(_Vector3 v1, _Vector3 v2) {
    float dot_product = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	dot_product *= dot_product;
    float magnitude_product = vector_length_pow2(v1) * vector_length_pow2(v2);
    if (magnitude_product == 0) return 0.0f;
    return dot_product / magnitude_product;
}

void calculate_tilt_angles(_Vector3 ref, _Vector3 current, float *theta_x, float *theta_y, float *theta_z) {
    *theta_x = atanf(current.y / current.z) - atanf(ref.y / ref.z);
    *theta_y = atanf(current.x / current.z) - atanf(ref.x / ref.z);
    *theta_z = atanf(current.x / current.y) - atanf(ref.x / ref.y);
}

/* Get accelerometer values and convert to m/s^2 */
static int get_accel_values(const struct device *dev, _Vector3 *accel)
{
	struct lis2dw12_data *data = dev->data;
	uint8_t raw_data[6];
	int16_t raw_x, raw_y, raw_z;
	int ret;

	/* Read all 6 bytes of accelerometer data */
	ret = lis2dw12_spi_read_reg(dev, LIS2DW12_REG_OUT_X_L, raw_data, 6);
	if (ret < 0) {
		LOG_ERR("Failed to read accelerometer data");
		return ret;
	}

	/* Combine bytes into 16-bit values */
	raw_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
	raw_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
	raw_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

	data->accel_x = raw_x;
	data->accel_y = raw_y;
	data->accel_z = raw_z;

	/* Convert to m/s^2 (assuming ±2g range, 14-bit resolution in low-power mode) */
	/* Sensitivity: 0.244 mg/LSB for ±2g range */
	/* 1 LSB = 0.244 mg = 0.000244 g = 0.000244 * 9.81 m/s^2 = 0.002394 m/s^2 */
	float scale = 0.002394f; /* For ±2g range, 14-bit */
	
	accel->x = (float)raw_x * scale;
	accel->y = (float)raw_y * scale;
	accel->z = (float)raw_z * scale;

	return 0;
}

/* GPIO interrupt callback */
static void lis2dw12_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb, uint32_t pins)
{
	struct lis2dw12_data *data = CONTAINER_OF(cb, struct lis2dw12_data, gpio_cb);
	
	/* Signal that data is ready */
	k_sem_give(&data->data_ready_sem);
}

/* Configure LIS2DW12 for data ready interrupt */
static int lis2dw12_config_interrupt(const struct device *dev)
{
	const struct lis2dw12_config *cfg = dev->config;
	struct lis2dw12_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure INT pin");
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, lis2dw12_gpio_callback,
			   BIT(cfg->int_gpio.pin));

	ret = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add GPIO callback");
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
					      GPIO_INT_EDGE_RISING);
	if (ret < 0) {
		LOG_ERR("Failed to configure GPIO interrupt");
		return ret;
	}

	/* Enable data ready interrupt on INT1 */
	ret = lis2dw12_write_byte(dev, LIS2DW12_REG_CTRL4_INT1, 
				  LIS2DW12_CTRL4_INT1_DRDY);
	if (ret < 0) {
		LOG_ERR("Failed to enable data ready interrupt");
		return ret;
	}

	LOG_DBG("Interrupt configured successfully");
	return 0;
}

/* Include detection logic */
#include "lis2dw12-sensor-logic.c"

/* Timer handlers */
static void refresh_current_pos_timer_handler_tilt(struct k_timer *timer)
{
	struct lis2dw12_data *data = CONTAINER_OF(timer, struct lis2dw12_data, refresh_current_pos_timer_tilt);
	LOG_DBG("Refreshing current position TILT");
	data->ref_acc_tilt.x = 0;
	data->ref_acc_tilt.y = 0;
	data->ref_acc_tilt.z = 0;
}

static void refresh_current_pos_timer_handler_move(struct k_timer *timer)
{
	struct lis2dw12_data *data = CONTAINER_OF(timer, struct lis2dw12_data, refresh_current_pos_timer_move);
	LOG_DBG("Refreshing current position MOVE");
	data->samples_count_move = 0;
	data->summary_acc_move.x = 0;
	data->summary_acc_move.y = 0;
	data->summary_acc_move.z = 0;
	data->ref_acc_move.x = 0;
	data->ref_acc_move.y = 0;
	data->ref_acc_move.z = 0;
	data->gravity = 0;
}

static void refresh_current_pos_timer_handler_move_disarmed(struct k_timer *timer)
{
	struct lis2dw12_data *data = CONTAINER_OF(timer, struct lis2dw12_data, refresh_current_pos_timer_move_disarmed);
	LOG_DBG("Refreshing current position MOVE DISARMED");
	data->samples_count_move_disarmed = 0;
	data->summary_acc_move_disarmed.x = 0;
	data->summary_acc_move_disarmed.y = 0;
	data->summary_acc_move_disarmed.z = 0;
	data->ref_acc_move_disarmed.x = 0;
	data->ref_acc_move_disarmed.y = 0;
	data->ref_acc_move_disarmed.z = 0;
	data->gravity_disarmed = 0;
	if (data->mode_move == ACCEL_SENSOR_MODE_DISARMED)
	{
		data->mode_move = ACCEL_SENSOR_MODE_ARMED;
		data->sampling_period_ms = MOVE_SENSOR_SAMPLE_TIME;
		LOG_INF("ARMED MOVE mode");
		if (data->disarm_move_handler && data->disarm_move_trigger) {
			data->disarm_move_handler(data->dev, data->disarm_move_trigger);
		}
	}
}

static void alarm_timer_handler_tilt(struct k_timer *timer)
{
	struct lis2dw12_data *data = CONTAINER_OF(timer, struct lis2dw12_data, alarm_timer_tilt);
	LOG_INF("TILT Alarm mode stopped");
	data->mode_tilt = ACCEL_SENSOR_MODE_ARMED;
	data->in_warn_alert_tilt = false;
	data->in_main_alert_tilt = false;
}

static void alarm_timer_handler_move(struct k_timer *timer)
{
	struct lis2dw12_data *data = CONTAINER_OF(timer, struct lis2dw12_data, alarm_timer_move);
	LOG_INF("MOVE Alarm mode stopped");
	data->mode_move = ACCEL_SENSOR_MODE_ARMED;
}

static void increase_sensivity_timer_handler_tilt(struct k_timer *timer)
{
	struct lis2dw12_data *data = CONTAINER_OF(timer, struct lis2dw12_data, increase_sensivity_timer_tilt);
	int prev_warn_zone = data->current_warn_zone_tilt;
	int prev_main_zone = data->current_main_zone_tilt;
	
	LOG_DBG("Trying increase sensivity TILT");
	
	if (data->mode_tilt == ACCEL_SENSOR_MODE_ARMED && (data->in_warn_alert_tilt || data->in_main_alert_tilt))
	{
		float pow_cos_theta = cospow2_between_vectors(data->ref_acc_tilt, data->last_acc_tilt);
		
		if (data->in_main_alert_tilt && data->main_zone_active_tilt)
		{
			while (data->current_main_zone_tilt > data->selected_main_zone_tilt)
			{
				if (data->main_zone_cos_pow2[data->current_main_zone_tilt] < pow_cos_theta) {
					data->current_main_zone_tilt--;
				} else {
					if (data->current_main_zone_tilt == 9) break;
					data->current_main_zone_tilt++;
					break;
				}
			}
		}
		
		if (data->current_main_zone_tilt == data->selected_main_zone_tilt && data->warn_zone_active_tilt)
		{
			while (data->current_warn_zone_tilt > data->selected_warn_zone_tilt)
			{
				if (data->warn_zone_cos_pow2[data->current_warn_zone_tilt] < pow_cos_theta) {
					data->current_warn_zone_tilt--;
				} else {
					if (data->current_warn_zone_tilt == 9) break;
					data->current_warn_zone_tilt++;
					break;
				}
			}
		}
	} else {
		k_timer_start(&data->increase_sensivity_timer_tilt, K_SECONDS(30), K_NO_WAIT);
		return;
	}

	if (data->current_main_zone_tilt != prev_main_zone){
		LOG_DBG("Main zone changed from %d to %d", prev_main_zone, data->current_main_zone_tilt);
		data->max_main_alert_level_tilt = false;
	}

	if (data->current_warn_zone_tilt != prev_warn_zone){
		LOG_DBG("Warn zone changed from %d to %d", prev_warn_zone, data->current_warn_zone_tilt);
		data->max_warn_alert_level_tilt = false;
	}

	if (data->current_warn_zone_tilt == data->selected_warn_zone_tilt) {
		data->in_warn_alert_tilt = false;
	}

	if (data->current_main_zone_tilt == data->selected_main_zone_tilt) {
		data->in_main_alert_tilt = false;
	}

	if (data->in_warn_alert_tilt || data->in_main_alert_tilt) {
		k_timer_start(&data->increase_sensivity_timer_tilt, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
	}
}

static void increase_sensivity_warn_timer_handler_move(struct k_timer *timer)
{
	struct lis2dw12_data *data = CONTAINER_OF(timer, struct lis2dw12_data, increase_sensivity_warn_timer_move);
	int prev_warn_zone = data->current_warn_zone_move;
	
	LOG_DBG("Trying increase warn MOVE sensivity");
	
	if (data->mode_move == ACCEL_SENSOR_MODE_ARMED)
	{
		if (data->current_warn_zone_move != data->selected_warn_zone_move)
		{
			data->current_warn_zone_move -= 1;
		}
	} else {
		k_timer_start(&data->increase_sensivity_warn_timer_move, K_SECONDS(30), K_NO_WAIT);
		return;
	}

	if (data->current_warn_zone_move != prev_warn_zone){
		LOG_DBG("Warn move zone changed from %d to %d", prev_warn_zone, data->current_warn_zone_move);
		data->max_warn_alert_level_move = false;
	}

	if (data->current_warn_zone_move != data->selected_warn_zone_move) {
		k_timer_start(&data->increase_sensivity_warn_timer_move, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
	}
}

static void increase_sensivity_main_timer_handler_move(struct k_timer *timer)
{
	struct lis2dw12_data *data = CONTAINER_OF(timer, struct lis2dw12_data, increase_sensivity_main_timer_move);
	int prev_main_zone = data->current_main_zone_move;
	
	LOG_DBG("Trying increase main MOVE sensivity");
	
	if (data->mode_move == ACCEL_SENSOR_MODE_ARMED)
	{
		if (data->current_main_zone_move != data->selected_main_zone_move)
		{
			data->current_main_zone_move -= 1;
		}
	} else {
		k_timer_start(&data->increase_sensivity_main_timer_move, K_SECONDS(30), K_NO_WAIT);
		return;
	}

	if (data->current_main_zone_move != prev_main_zone){
		LOG_DBG("Main move zone changed from %d to %d", prev_main_zone, data->current_main_zone_move);
		data->max_main_alert_level_move = false;
	}

	if (data->current_main_zone_move != data->selected_main_zone_move) {
		k_timer_start(&data->increase_sensivity_main_timer_move, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
	}
}

/* Include initialization functions */
#include "lis2dw12-sensor-init.c"

/* Device initialization */
static int lis2dw12_init(const struct device *dev)
{
	const struct lis2dw12_config *cfg = dev->config;
	struct lis2dw12_data *data = dev->data;
	uint8_t chip_id;
	int ret;

	LOG_INF("Initializing LIS2DW12 accelerometer sensor (%s)", dev->name);

	data->dev = dev;

	/* Check SPI device */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	/* Verify WHO_AM_I */
	ret = lis2dw12_read_byte(dev, LIS2DW12_REG_WHO_AM_I, &chip_id);
	if (ret < 0) {
		LOG_ERR("Failed to read WHO_AM_I register");
		return ret;
	}

	if (chip_id != LIS2DW12_WHO_AM_I_VALUE) {
		LOG_ERR("Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, LIS2DW12_WHO_AM_I_VALUE);
		return -ENODEV;
	}

	LOG_INF("LIS2DW12 chip ID verified: 0x%02X", chip_id);

	/* Software reset */
	ret = lis2dw12_write_byte(dev, LIS2DW12_REG_CTRL2, LIS2DW12_CTRL2_SOFT_RESET);
	if (ret < 0) {
		LOG_ERR("Failed to perform software reset");
		return ret;
	}

	k_msleep(10); /* Wait for reset to complete */

	/* Configure CTRL2: Enable BDU (Block Data Update) and auto-increment */
	ret = lis2dw12_write_byte(dev, LIS2DW12_REG_CTRL2, 
	                          LIS2DW12_CTRL2_BDU | LIS2DW12_CTRL2_IF_ADD_INC);
	if (ret < 0) {
		LOG_ERR("Failed to configure CTRL2");
		return ret;
	}

	/* Configure CTRL6: ±2g range, low-noise mode */
	/* FS = 00 (±2g), FDS = 0, LOW_NOISE = 1 */
	ret = lis2dw12_write_byte(dev, LIS2DW12_REG_CTRL6, 
	                          LIS2DW12_CTRL6_LOW_NOISE | (0 << LIS2DW12_CTRL6_FS_SHIFT));
	if (ret < 0) {
		LOG_ERR("Failed to configure CTRL6");
		return ret;
	}

	/* Configure CTRL1: Set ODR to 100 Hz, Low-power mode 1 */
	/* ODR = 0101 (100 Hz), MODE = 00 (Low-power), LP_MODE = 01 (12-bit) */
	uint8_t ctrl1_val = (5 << LIS2DW12_CTRL1_ODR_SHIFT) | 
	                    (0 << LIS2DW12_CTRL1_MODE_SHIFT) | 
	                    (1 << LIS2DW12_CTRL1_LP_MODE_SHIFT);
	ret = lis2dw12_write_byte(dev, LIS2DW12_REG_CTRL1, ctrl1_val);
	if (ret < 0) {
		LOG_ERR("Failed to configure CTRL1");
		return ret;
	}

	LOG_DBG("LIS2DW12 configured: ±2g range, 100 Hz ODR, low-power mode");

	/* Initialize semaphore for data ready */
	k_sem_init(&data->data_ready_sem, 0, 1);

	/* Configure interrupt */
	ret = lis2dw12_config_interrupt(dev);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt");
		return ret;
	}

	/* Initialize tilt detection */
	init_warn_zones_tilt(dev);
	set_warn_zone_tilt(dev, 5);
	change_main_zone_tilt(dev, 5);

	LOG_DBG("Starting periodic measurements (%d ms)", data->sampling_period_ms);
	
	/* Initialize tilt detection state */
	data->in_warn_alert_tilt = false;
	data->in_main_alert_tilt = false;
	data->max_warn_alert_level_tilt = false;
	data->max_main_alert_level_tilt = false;
	data->warn_zone_active_tilt = true;
	data->main_zone_active_tilt = true;
	data->mode_tilt = ACCEL_SENSOR_MODE_DISARMED;
	
	k_timer_init(&data->refresh_current_pos_timer_tilt, refresh_current_pos_timer_handler_tilt, NULL);
	k_timer_init(&data->increase_sensivity_timer_tilt, increase_sensivity_timer_handler_tilt, NULL);
	k_timer_init(&data->alarm_timer_tilt, alarm_timer_handler_tilt, NULL);
	
	/* Initialize movement detection state */
	create_warn_zones_move(dev);
	create_main_zones_move(dev, 5);
	data->max_warn_alert_level_move = false;
	data->max_main_alert_level_move = false;
	data->warn_zone_active_move = true;
	data->main_zone_active_move = true;
	data->mode_move = ACCEL_SENSOR_MODE_DISARMED;
	data->selected_warn_zone_move = 5;
	data->current_warn_zone_move = 5;
	data->selected_main_zone_move = 5;
	data->current_main_zone_move = 5;
	
	k_timer_init(&data->refresh_current_pos_timer_move, refresh_current_pos_timer_handler_move, NULL);
	k_timer_init(&data->increase_sensivity_warn_timer_move, increase_sensivity_warn_timer_handler_move, NULL);
	k_timer_init(&data->increase_sensivity_main_timer_move, increase_sensivity_main_timer_handler_move, NULL);
	k_timer_init(&data->alarm_timer_move, alarm_timer_handler_move, NULL);
	k_timer_init(&data->refresh_current_pos_timer_move_disarmed, refresh_current_pos_timer_handler_move_disarmed, NULL);
	
	data->samples_count_move_disarmed = 0;
	data->summary_acc_move_disarmed.x = 0;
	data->summary_acc_move_disarmed.y = 0;
	data->summary_acc_move_disarmed.z = 0;
	data->samples_count_move = 0;
	data->summary_acc_move.x = 0;
	data->summary_acc_move.y = 0;
	data->summary_acc_move.z = 0;
	
	/* Start periodic work */
	k_work_init_delayable(&data->dwork, accel_work_handler);
	k_work_schedule(&data->dwork, K_MSEC(data->sampling_period_ms));

	LOG_INF("LIS2DW12 initialization complete");
	return 0;
}

#define LIS2DW12_DEFINE(inst)                                                  \
	static struct lis2dw12_data lis2dw12_data_##inst = {                  \
		.sampling_period_ms = DT_INST_PROP_OR(inst, sampling_period_ms, 1000), \
	};                                                                     \
	                                                                       \
	static const struct lis2dw12_config lis2dw12_config_##inst = {        \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0), \
		.int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),              \
	};                                                                     \
	                                                                       \
	DEVICE_DT_INST_DEFINE(inst, lis2dw12_init, NULL,                      \
			      &lis2dw12_data_##inst, &lis2dw12_config_##inst, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &driver_api);

DT_INST_FOREACH_STATUS_OKAY(LIS2DW12_DEFINE)

