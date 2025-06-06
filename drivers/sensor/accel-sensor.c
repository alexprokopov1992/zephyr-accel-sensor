#define DT_DRV_COMPAT zephyr_accel_sensor

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

#include "accel-sensor.h"
#include <math.h>

// LOG_MODULE_REGISTER(accel_sensor, LOG_LEVEL_DBG);
LOG_MODULE_REGISTER(accel_sensor, CONFIG_SENSOR_LOG_LEVEL);

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

static float warn_zone_start_angle = 1.0;
static float warn_zone_step_angle = 2.0/9.0;
static float max_angle = 10.00;
static const float cos_pow_0_5  = 0.999961923;

static float border_move = 0.0005;

static float warn_zone_accel_mult = 0.001;
static float warn_zone_step_accel_mult_step = 0.001;
static float main_zone_max_mult = 0.1;

// Функція для обчислення довжини вектора
float vector_length(_Vector3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

float vector_length_pow2(_Vector3 v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

// Функція для обчислення косинуса кута між двома векторами
float angle_between_vectors(_Vector3 v1, _Vector3 v2) {
    float dot_product = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    float magnitude_product = vector_length(v1) * vector_length(v2);
    if (magnitude_product == 0) return 0.0f; // уникнення ділення на нуль
    return acosf(dot_product / magnitude_product); // кут у радіанах
}

float cospow2_between_vectors(_Vector3 v1, _Vector3 v2) {
    float dot_product = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	dot_product *= dot_product;
    float magnitude_product = vector_length_pow2(v1) * vector_length_pow2(v2);
    if (magnitude_product == 0) return 0.0f; // уникнення ділення на нуль
    return dot_product / magnitude_product; // кут у радіанах
}

// Функція для обчислення кутів нахилу по осях
void calculate_tilt_angles(_Vector3 ref, _Vector3 current, float *theta_x, float *theta_y, float *theta_z) {
    *theta_x = atanf(current.y / current.z) - atanf(ref.y / ref.z);
    *theta_y = atanf(current.x / current.z) - atanf(ref.x / ref.z);
    *theta_z = atanf(current.x / current.y) - atanf(ref.x / ref.y);
}

int get_mma8652_val(const struct device *dev, struct sensor_value *val)
{
	if (sensor_sample_fetch(dev)) {
		printk("Failed to fetch sample for device %s\n",
		       dev->name);
		return -1;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, &val[0])) {
		return -1;
	}

	return 0;
}


static int write_reg(const struct device *i2c_dev, uint8_t reg, uint8_t data) {
    return i2c_reg_write_byte(i2c_dev, MMA8652_ADDR, reg, data);
}

static void mma8652fc_config_motion()
{
	const struct device *i2c_dev = device_get_binding("I2C_1");

    if (!i2c_dev) {
        LOG_ERR("Failed to get I2C device");
        return;
    }

	uint8_t int_src;

	i2c_reg_read_byte(i2c_dev, MMA8652_ADDR, CTRL_REG1, &int_src);
	int_src &= ~0x01;
    write_reg(i2c_dev, CTRL_REG1, int_src);               // Standby

    write_reg(i2c_dev, FF_MT_CFG, 0b11111000);         // Motion detection on XYZ
	// write_reg(i2c_dev, F_SETUP, (0b10 << 6) | 16);               // Clear FF_MT_SRC register
	write_reg(i2c_dev, F_SETUP, 0x00);               // Clear FF_MT_SRC register
    write_reg(i2c_dev, FF_MT_THS, 0x01);               // ~0.063g
    write_reg(i2c_dev, FF_MT_COUNT, 0x04);             // debounce count
    write_reg(i2c_dev, CTRL_REG4, 0x04);               // Enable FF_MT interrupt
    write_reg(i2c_dev, CTRL_REG5, 0x00);               // Route FF_MT to INT2

	i2c_reg_read_byte(i2c_dev, MMA8652_ADDR, CTRL_REG1, &int_src);
	int_src |= 0x01;
    write_reg(i2c_dev, CTRL_REG1, int_src);               // Active mode

    k_msleep(10); // let it settle

    i2c_reg_read_byte(i2c_dev, MMA8652_ADDR, 0x0C, &int_src);
    printk("INT_SOURCE: 0x%02X\n", int_src);

	// i2c_reg_read_byte(i2c_dev, MMA8652_ADDR, CTRL_REG5, &int_src);
    // printk("Value is: 0x%02X\n", int_src);

	// i2c_reg_read_byte(i2c_dev, MMA8652_ADDR, F_SETUP, &int_src);
    // printk("Value is: 0x%02X\n", int_src);

	// i2c_reg_read_byte(i2c_dev, MMA8652_ADDR, FF_MT_COUNT, &int_src);
    // printk("Value is: 0x%02X\n", int_src);
}


static void coarsering_tilt(struct accel_sensor_data *data, int level)
{
	if (level == 0)
	{
		if (data->current_warn_zone_tilt == 9){
			data->max_warn_alert_level_tilt = true;
			LOG_DBG("Max warn level reached %d", data->current_warn_zone_tilt);
		} else if (data->warn_zone_cos_pow2[data->current_warn_zone_tilt+1] < data->main_zone_cos_pow2[data->selected_main_zone_tilt])
		{
			data->max_warn_alert_level_tilt = true;
			LOG_DBG("Max warn level reached %d", data->current_warn_zone_tilt);
		} else {
			data->current_warn_zone_tilt++;
			LOG_DBG("Coarsering WARN_ZONE to %d", data->current_warn_zone_tilt);
		}
	} else {
		if (data->current_main_zone_tilt == 9){
			data->max_warn_alert_level_tilt = true;
			data->max_main_alert_level_tilt = true;
			while (data->warn_zone_cos_pow2[data->current_warn_zone_tilt] > data->main_zone_cos_pow2[data->selected_main_zone_tilt])
			{
				if (data->current_warn_zone_tilt == 9) break;
				data->current_warn_zone_tilt++;
			}
			if (data->warn_zone_cos_pow2[data->current_warn_zone_tilt] <= data->main_zone_cos_pow2[data->selected_main_zone_tilt] && data->current_warn_zone_tilt > 0) data->current_warn_zone_tilt--;
			LOG_DBG("Coarsering WARN_ZONE to %d", data->current_warn_zone_tilt);
			LOG_DBG("Max main level reached %d", data->current_main_zone_tilt);
		} else {
			data->current_main_zone_tilt += 1;
			data->max_warn_alert_level_tilt = true;
			while (data->warn_zone_cos_pow2[data->current_warn_zone_tilt] > data->main_zone_cos_pow2[data->selected_main_zone_tilt])
			{
				if (data->current_warn_zone_tilt == 9) break;
				data->current_warn_zone_tilt++;
			}
			if (data->warn_zone_cos_pow2[data->current_warn_zone_tilt] <= data->main_zone_cos_pow2[data->selected_main_zone_tilt] && data->current_warn_zone_tilt > 0) data->current_warn_zone_tilt--;
			LOG_DBG("Max warn level reached %d", data->current_warn_zone_tilt);
			LOG_DBG("Coarsering MAIN_ZONE to %d", data->current_main_zone_tilt);
		}
	}
}

static void coarsering_move(struct accel_sensor_data *data, int level)
{
	if (level == 0)
	{
		if (data->current_warn_zone_move == 9){
			data->max_warn_alert_level_move = true;
			LOG_DBG("Max MOVE warn level reached %d", data->current_warn_zone_move);
		} else if (data->warn_zone_move[data->current_warn_zone_move+1] > data->main_zone_move[data->selected_main_zone_move])
		{
			data->max_warn_alert_level_move = true;
			LOG_DBG("Max MOVE warn level reached %d", data->current_warn_zone_move);
		} else {
			data->current_warn_zone_move++;
			LOG_DBG("Coarsering WARN_ZONE_MOVE to %d", data->current_warn_zone_move);
		}
	} else {
		if (data->current_main_zone_move == 9){
			data->max_main_alert_level_move = true;
			LOG_DBG("Max MOVE main level reached %d", data->current_main_zone_move);
		} else {
			data->current_main_zone_move += 1;
			LOG_DBG("Coarsering MAIN_ZONE_MOVE to %d", data->current_main_zone_move);
		}
	}
}

static bool both_mode_disarmed(struct accel_sensor_data *data)
{
	if (data->mode_tilt == ACCEL_SENSOR_MODE_DISARMED && data->mode_move == ACCEL_SENSOR_MODE_DISARMED)
	{
		return true;
	}
	return false;
}

static void process_disarmed_move(struct accel_sensor_data *data, const struct device *dev, _Vector3 current_acc, int64_t current_time)
{
	if (data->samples_count_move_disarmed >= MOVE_SENSOR_SAMPLE_COUNT){
			data->last_acc_move_disarmed.x = data->summary_acc_move_disarmed.x / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->last_acc_move_disarmed.y = data->summary_acc_move_disarmed.y / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->last_acc_move_disarmed.z = data->summary_acc_move_disarmed.z / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->summary_acc_move_disarmed.x = 0;
			data->summary_acc_move_disarmed.y = 0;
			data->summary_acc_move_disarmed.z = 0;
			data->samples_count_move_disarmed = 0;

			float acc_len = vector_length(data->last_acc_move_disarmed);
			_Vector3 accelerate = {data->last_acc_move_disarmed.x - data->ref_acc_move_disarmed.x, data->last_acc_move_disarmed.y - data->ref_acc_move_disarmed.y, data->last_acc_move_disarmed.z - data->ref_acc_move_disarmed.z};
			float accel = vector_length(accelerate);

			if (data->gravity_disarmed > 0)
			{
				float change = (data->gravity_disarmed - acc_len)/data->gravity_disarmed;
				if (change < border_move && change > -border_move)
				{
					data->gravity_disarmed = acc_len;
					data->ref_acc_move_disarmed = data->last_acc_move_disarmed;
				} else {
					if (accel > data->warn_zone_move[data->selected_warn_zone_move]*data->gravity_disarmed) {
						if (current_time - data->last_trigger_time_disarmed_move > MIN_WARN_INTERVAL) {
							LOG_DBG("Disarmed move triggered");
							LOG_DBG("Move sensor value X:%10.6f Y:%10.6f Z:%10.6f accel: %10.6f gravity: %10.6f last_acc: %10.6f", (double)data->last_acc_move_disarmed.x, (double)data->last_acc_move_disarmed.y, (double)data->last_acc_move_disarmed.z, (double)accel, (double)data->gravity_disarmed, (double)acc_len);
							data->last_trigger_time_disarmed_move = current_time;
							data->disarm_move_handler(dev, data->disarm_move_trigger);
						}
					}
				}
			}

		} else {
			data->summary_acc_move_disarmed.x += current_acc.x;
			data->summary_acc_move_disarmed.y += current_acc.y;
			data->summary_acc_move_disarmed.z += current_acc.z;
			data->samples_count_move_disarmed++;
		}
}

static bool process_tilt_mode(struct accel_sensor_data *data, const struct device *dev, _Vector3 current_acc, int64_t current_time)
{
	float pow_cos_theta = cospow2_between_vectors(data->ref_acc_tilt, current_acc);
	if (pow_cos_theta == 0) {
		k_work_schedule(&data->dwork, K_MSEC(data->sampling_period_ms));
		return false;
	}

	if (pow_cos_theta < data->main_zone_cos_pow2[data->current_main_zone_tilt] && data->main_zone_active_tilt)
	{
		if (!data->max_main_alert_level_tilt){
			data->mode_tilt = ACCEL_SENSOR_MODE_ALARM;
			data->in_warn_alert_tilt = true;
			data->in_main_alert_tilt = true;
			coarsering_tilt(data, 1);
			data->last_trigger_time_main_tilt = current_time;
			LOG_DBG("MAIN TRIGGER TILT");
			data->main_handler_tilt(dev, data->main_trigger_tilt);
		}
		k_timer_start(&data->increase_sensivity_timer_tilt, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
	} else if (pow_cos_theta < data->warn_zone_cos_pow2[data->current_warn_zone_tilt] && data->warn_zone_active_tilt) {
		if (!data->max_warn_alert_level_tilt) {
			if (current_time - data->last_trigger_time_warn_tilt > MIN_WARN_INTERVAL) {
				data->in_warn_alert_tilt = true;
				coarsering_tilt(data, 0);
				data->last_trigger_time_warn_tilt = current_time;
				LOG_DBG("WARN TRIGGER TILT");
				data->warn_handler_tilt(dev, data->warn_trigger_tilt);
			}
			k_timer_start(&data->increase_sensivity_timer_tilt, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
		}	
	}
	return true;
}

static void process_move_mode(struct accel_sensor_data *data, const struct device *dev, _Vector3 current_acc, int64_t current_time)
{
	if (data->samples_count_move >= MOVE_SENSOR_SAMPLE_COUNT){
			data->last_acc_move.x = data->summary_acc_move.x / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->last_acc_move.y = data->summary_acc_move.y / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->last_acc_move.z = data->summary_acc_move.z / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->summary_acc_move.x = 0;
			data->summary_acc_move.y = 0;
			data->summary_acc_move.z = 0;
			data->samples_count_move = 0;

			if (data->mode_move == ACCEL_SENSOR_MODE_ARMED)
			{
				float acc_len = vector_length(data->last_acc_move);
				_Vector3 accelerate = {data->last_acc_move.x - data->ref_acc_move.x, data->last_acc_move.y - data->ref_acc_move.y, data->last_acc_move.z - data->ref_acc_move.z};
				float accel = vector_length(accelerate);
				if (data->gravity > 0)
				{
					float change = (data->gravity - acc_len)/data->gravity;
					if (change < border_move && change > -border_move)
					{
						data->gravity = acc_len;
						data->ref_acc_move = data->last_acc_move;
					} else {
						if (accel > data->main_zone_move[data->current_main_zone_move]*data->gravity && data->main_zone_active_move)
						{
							if (!data->max_main_alert_level_move){
								data->mode_move = ACCEL_SENSOR_MODE_ALARM;
								coarsering_move(data, 1);
								LOG_DBG("Move Main zone move triggered");
								data->last_trigger_time_main_move = current_time;
								data->main_handler_move(dev, data->main_trigger_move);
							}
							k_timer_start(&data->increase_sensivity_main_timer_move, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
						} else {
							if (accel > data->warn_zone_move[data->current_warn_zone_move]*data->gravity && data->warn_zone_active_move) {
								if (!data->max_warn_alert_level_move) {
									if (current_time - data->last_trigger_time_warn_move > MIN_WARN_INTERVAL) {
										coarsering_move(data, 0);
										LOG_DBG("Move Warn zone move triggered");
										data->last_trigger_time_warn_move = current_time;
										data->warn_handler_move(dev, data->warn_trigger_move);
									}
								}
								k_timer_start(&data->increase_sensivity_warn_timer_move, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
							}
						}
					}

					// LOG_DBG("Move sensor value X:%10.6f Y:%10.6f Z:%10.6f accel: %10.6f gravity: %10.6f last_acc: %10.6f", (double)data->last_acc_move.x, (double)data->last_acc_move.y, (double)data->last_acc_move.z, (double)accel, (double)data->gravity, (double)acc_len);
				}
			}	
		} else {
			data->summary_acc_move.x += current_acc.x;
			data->summary_acc_move.y += current_acc.y;
			data->summary_acc_move.z += current_acc.z;
			data->samples_count_move++;
		}
}

static void adc_vbus_work_handler_refactor(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct accel_sensor_data *data = CONTAINER_OF(delayable, struct accel_sensor_data, dwork);
	const struct device *dev = data->accel_dev;

	struct sensor_value val[3];
    get_mma8652_val(dev, val);

	 _Vector3 current_acc = {
        sensor_value_to_double(&val[0]),
        sensor_value_to_double(&val[1]),
        sensor_value_to_double(&val[2])
    };
    data->last_acc_tilt = current_acc;

	int64_t current_time = k_uptime_get();

	if (both_mode_disarmed(data))
	{
		process_disarmed_move(data, dev, current_acc, current_time);
		k_work_schedule(&data->dwork, K_MSEC(MOVE_SENSOR_SAMPLE_TIME));
		return;
	}

	bool allowed = false;
	if (data->mode_move == ACCEL_SENSOR_MODE_ARMED) {
		data->skip_counter++;
		if (data->skip_counter > 9) {
			data->skip_counter = 0;
			allowed = true;
		} else {
			allowed = false;
		}
	} else {
		allowed = true;
	}

	if (data->mode_tilt == ACCEL_SENSOR_MODE_ARMED && allowed)
	{
		if (!process_tilt_mode(data, dev, current_acc, current_time)) return;
	}

	if (data->mode_move == ACCEL_SENSOR_MODE_ARMED || data->mode_move == ACCEL_SENSOR_MODE_ALARM){
		process_move_mode(data, dev, current_acc, current_time);
	}

	k_work_schedule(&data->dwork, K_MSEC(data->sampling_period_ms));
}

static void adc_vbus_work_handler(struct k_work *work)
{
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct accel_sensor_data *data = CONTAINER_OF(delayable, struct accel_sensor_data, dwork);

	const struct device *dev = data->accel_dev;

	struct sensor_value val[3];

    get_mma8652_val(dev, val);
    float ax = sensor_value_to_double(&val[0]);
    float ay = sensor_value_to_double(&val[1]);
    float az = sensor_value_to_double(&val[2]);
    _Vector3 current_acc = {ax, ay, az};
    data->last_acc_tilt = current_acc;

	if (data->mode_tilt == ACCEL_SENSOR_MODE_DISARMED && data->mode_move == ACCEL_SENSOR_MODE_DISARMED) {
		int64_t current_time = k_uptime_get();
		if (data->samples_count_move_disarmed >= MOVE_SENSOR_SAMPLE_COUNT){
			data->last_acc_move_disarmed.x = data->summary_acc_move_disarmed.x / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->last_acc_move_disarmed.y = data->summary_acc_move_disarmed.y / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->last_acc_move_disarmed.z = data->summary_acc_move_disarmed.z / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->summary_acc_move_disarmed.x = 0;
			data->summary_acc_move_disarmed.y = 0;
			data->summary_acc_move_disarmed.z = 0;
			data->samples_count_move_disarmed = 0;

			float acc_len = vector_length(data->last_acc_move_disarmed);
			_Vector3 accelerate = {data->last_acc_move_disarmed.x - data->ref_acc_move_disarmed.x, data->last_acc_move_disarmed.y - data->ref_acc_move_disarmed.y, data->last_acc_move_disarmed.z - data->ref_acc_move_disarmed.z};
			float accel = vector_length(accelerate);

			if (data->gravity_disarmed > 0)
			{
				float change = (data->gravity_disarmed - acc_len)/data->gravity_disarmed;
				if (change < border_move && change > -border_move)
				{
					data->gravity_disarmed = acc_len;
					data->ref_acc_move_disarmed = data->last_acc_move_disarmed;
				} else {
					if (accel > data->warn_zone_move[data->selected_warn_zone_move]*data->gravity_disarmed) {
						if (current_time - data->last_trigger_time_disarmed_move > MIN_WARN_INTERVAL) {
							LOG_DBG("Disarmed move triggered");
							LOG_DBG("Move sensor value X:%10.6f Y:%10.6f Z:%10.6f accel: %10.6f gravity: %10.6f last_acc: %10.6f", (double)data->last_acc_move_disarmed.x, (double)data->last_acc_move_disarmed.y, (double)data->last_acc_move_disarmed.z, (double)accel, (double)data->gravity_disarmed, (double)acc_len);
							data->last_trigger_time_disarmed_move = current_time;
							data->disarm_move_handler(dev, data->disarm_move_trigger);
						}
					}
				}
			}

		} else {
			data->summary_acc_move_disarmed.x += ax;
			data->summary_acc_move_disarmed.y += ay;
			data->summary_acc_move_disarmed.z += az;
			data->samples_count_move_disarmed++;
		}
		k_work_schedule(&data->dwork, K_MSEC(MOVE_SENSOR_SAMPLE_TIME));
		return;
	}

	bool allowed = false;
	if (data->mode_move == ACCEL_SENSOR_MODE_ARMED) {
		data->skip_counter++;
		if (data->skip_counter > 9) {
			data->skip_counter = 0;
			allowed = true;
		} else {
			allowed = false;
		}
	} else {
		allowed = true;
	}

	if (data->mode_tilt == ACCEL_SENSOR_MODE_ARMED && allowed)
	{
		float pow_cos_theta = cospow2_between_vectors(data->ref_acc_tilt, current_acc);
		if (pow_cos_theta == 0) {
			k_work_schedule(&data->dwork, K_MSEC(data->sampling_period_ms));
			return;
		}

		int64_t current_time = k_uptime_get();

		if (pow_cos_theta < data->main_zone_cos_pow2[data->current_main_zone_tilt] && data->main_zone_active_tilt)
		{
			if (!data->max_main_alert_level_tilt){
				data->mode_tilt = ACCEL_SENSOR_MODE_ALARM;
				data->in_warn_alert_tilt = true;
				data->in_main_alert_tilt = true;
				coarsering_tilt(data, 1);
				data->last_trigger_time_main_tilt = current_time;
				LOG_DBG("MAIN TRIGGER TILT");
				data->main_handler_tilt(dev, data->main_trigger_tilt);
			}
			k_timer_start(&data->increase_sensivity_timer_tilt, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
		} else if (pow_cos_theta < data->warn_zone_cos_pow2[data->current_warn_zone_tilt] && data->warn_zone_active_tilt) {
			if (!data->max_warn_alert_level_tilt) {
				if (current_time - data->last_trigger_time_warn_tilt > MIN_WARN_INTERVAL) {
					data->in_warn_alert_tilt = true;
					coarsering_tilt(data, 0);
					data->last_trigger_time_warn_tilt = current_time;
					LOG_DBG("WARN TRIGGER TILT");
					data->warn_handler_tilt(dev, data->warn_trigger_tilt);
				}
				k_timer_start(&data->increase_sensivity_timer_tilt, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
			}	
		}
		// LOG_DBG(
		// 	"Current values: X:%.4f Y:%.4f Z:%.4f"
		// 	" pow cos theta: %.8f"
		// 	, (double)ax, (double)ay, (double)az
		// 	, (double)(pow_cos_theta)
		// );
	}

	if (data->mode_move == ACCEL_SENSOR_MODE_ARMED || data->mode_move == ACCEL_SENSOR_MODE_ALARM){

		int64_t current_time = k_uptime_get();
		if (data->samples_count_move >= MOVE_SENSOR_SAMPLE_COUNT){
			data->last_acc_move.x = data->summary_acc_move.x / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->last_acc_move.y = data->summary_acc_move.y / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->last_acc_move.z = data->summary_acc_move.z / (float)MOVE_SENSOR_SAMPLE_COUNT;
			data->summary_acc_move.x = 0;
			data->summary_acc_move.y = 0;
			data->summary_acc_move.z = 0;
			data->samples_count_move = 0;

			if (data->mode_move == ACCEL_SENSOR_MODE_ARMED)
			{
				float acc_len = vector_length(data->last_acc_move);
				_Vector3 accelerate = {data->last_acc_move.x - data->ref_acc_move.x, data->last_acc_move.y - data->ref_acc_move.y, data->last_acc_move.z - data->ref_acc_move.z};
				float accel = vector_length(accelerate);
				if (data->gravity > 0)
				{
					float change = (data->gravity - acc_len)/data->gravity;
					if (change < border_move && change > -border_move)
					{
						data->gravity = acc_len;
						data->ref_acc_move = data->last_acc_move;
					} else {
						if (accel > data->main_zone_move[data->current_main_zone_move]*data->gravity && data->main_zone_active_move)
						{
							if (!data->max_main_alert_level_move){
								data->mode_move = ACCEL_SENSOR_MODE_ALARM;
								coarsering_move(data, 1);
								LOG_DBG("Move Main zone move triggered");
								data->last_trigger_time_main_move = current_time;
								data->main_handler_move(dev, data->main_trigger_move);
							}
							k_timer_start(&data->increase_sensivity_main_timer_move, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
						} else {
							if (accel > data->warn_zone_move[data->current_warn_zone_move]*data->gravity && data->warn_zone_active_move) {
								if (!data->max_warn_alert_level_move) {
									if (current_time - data->last_trigger_time_warn_move > MIN_WARN_INTERVAL) {
										coarsering_move(data, 0);
										LOG_DBG("Move Warn zone move triggered");
										data->last_trigger_time_warn_move = current_time;
										data->warn_handler_move(dev, data->warn_trigger_move);
									}
								}
								k_timer_start(&data->increase_sensivity_warn_timer_move, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
							}
						}
					}

					// LOG_DBG("Move sensor value X:%10.6f Y:%10.6f Z:%10.6f accel: %10.6f gravity: %10.6f last_acc: %10.6f", (double)data->last_acc_move.x, (double)data->last_acc_move.y, (double)data->last_acc_move.z, (double)accel, (double)data->gravity, (double)acc_len);
				}
			}	
		} else {
			data->summary_acc_move.x += ax;
			data->summary_acc_move.y += ay;
			data->summary_acc_move.z += az;
			data->samples_count_move++;
		}
	}

	// TODO: Add callback
	#if 0
    	balance_overwrite_value((100.0f * theta * (180.0f / M_PIf)));
	#endif
    // force_send_state();

    // LOG_ERR(
    //     "Move sensor value X:%10.3f Y:%10.3f X:%10.3f",
    //     sensor_value_to_double(&val[0]),
    //     sensor_value_to_double(&val[1]),
    //     sensor_value_to_double(&val[2])
    //     // val[0].val1, val[1].val1, val[2].val1
    // );
    // adc_vbus_process();



    k_work_schedule(&data->dwork, K_MSEC(data->sampling_period_ms));
}

static void init_warn_zones_tilt(const struct device *dev)
{
	struct accel_sensor_data *data = dev->data;
	data->warn_zone_cos_pow2[0] = warn_zone_start_angle;
	for (int i = 1; i < 10; i++)
	{
		data->warn_zone_cos_pow2[i] = data->warn_zone_cos_pow2[i-1] + warn_zone_step_angle;
	}

	// LOG_DBG("Warning zone angles (degrees):");
	for (int i = 0; i < 10; i++)
	{
		// LOG_DBG("Angle[%d]: %d°", i, (int)(data->warn_zone_cos_pow2[i]*100));
		float angle_rad = data->warn_zone_cos_pow2[i] * (M_PIf / 180.0f);
		data->warn_zone_cos_pow2[i] = cosf(angle_rad) * cosf(angle_rad);
	}
}

static void create_warn_zones_move(const struct device *dev)
{
	struct accel_sensor_data *data = dev->data;
	for (int i = 0; i < 10; i++)
	{
		data->warn_zone_move[i] = warn_zone_accel_mult + warn_zone_step_accel_mult_step * i;
	}
	
}

static void create_main_zones_move(const struct device *dev, int warn_zone)
{
	struct accel_sensor_data *data = dev->data;
	float start_mult = warn_zone_step_accel_mult_step * (warn_zone + 2);
	float step = (main_zone_max_mult - start_mult) / (float)9.0;
	for (int i = 0; i < 10; i++)
	{
		data->main_zone_move[i] = start_mult + step * i;
	}
}

static void create_main_zones_tilt(const struct device *dev, int warn_zone)
{
	struct accel_sensor_data *data = dev->data;
	data->main_zone_cos_pow2[0] = warn_zone_start_angle + (float)(warn_zone + 1) * warn_zone_step_angle;
	float step = (max_angle - data->main_zone_cos_pow2[0]) / (float)9.0;
	for (int i = 1; i < 10; i++)
	{
		data->main_zone_cos_pow2[i] = data->main_zone_cos_pow2[i-1] + step;
	}

	// LOG_DBG("Main zone angles (degrees):");
	for (int i = 0; i < 10; i++)
	{
		// LOG_DBG("Angle[%d]: %d°", i, (int)(data->main_zone_cos_pow2[i]*100));
		float angle_rad = data->main_zone_cos_pow2[i] * (M_PIf / 180.0f);
		data->main_zone_cos_pow2[i] = cosf(angle_rad) * cosf(angle_rad);
	}
}

static void set_warn_zone_tilt(const struct device *dev, int zone)
{
	struct accel_sensor_data *data = dev->data;
	data->selected_warn_zone_tilt = zone;
	data->current_warn_zone_tilt = zone;
	create_main_zones_tilt(dev, zone);
	data->current_main_zone_tilt = data->selected_main_zone_tilt;
}

static void change_main_zone_tilt(const struct device *dev, int zone)
{
	struct accel_sensor_data *data = dev->data;
	data->current_main_zone_tilt = zone;
	data->selected_main_zone_tilt = zone;
	data->current_warn_zone_tilt = data->selected_warn_zone_tilt;
}

// static void change_warn_zone(const struct device *dev, int zone)
// {
// 	struct accel_sensor_data *data = dev->data;
// 	data->selected_warn_zone = zone;
// 	data->current_warn_zone = zone;
// }

// API
static int _save_current_positoin_as_reference(const struct device *dev)
{
	struct accel_sensor_data *data = dev->data;
	data->ref_acc_tilt = data->last_acc_tilt;
	return 0;
}

static void refresh_current_pos_timer_handler_tilt(struct k_timer *timer)
{
    struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, refresh_current_pos_timer_tilt);
	if (data->mode_tilt == ACCEL_SENSOR_MODE_ARMED)
	{
		LOG_DBG("Refreshing ref_acc_tilt");
		if (!data->in_warn_alert_tilt && !data->in_main_alert_tilt)
		{
			float pow_cos_theta = cospow2_between_vectors(data->ref_acc_tilt, data->last_acc_tilt);
			if (pow_cos_theta >  cos_pow_0_5 || pow_cos_theta == 0)
			{
				data->ref_acc_tilt = data->last_acc_tilt;
				LOG_DBG("ref_acc_tilt Refreshed");
			}  
			LOG_DBG("cosinuses: %.8f %.8f", (double)pow_cos_theta, (double)cos_pow_0_5); 
		}
	} else {
		k_timer_start(&data->refresh_current_pos_timer_tilt, K_SECONDS(5), K_NO_WAIT);
		return;
	}
	k_timer_start(&data->refresh_current_pos_timer_tilt, K_SECONDS(REFRESH_POS_TIME), K_NO_WAIT);
}

static void refresh_current_pos_timer_handler_move(struct k_timer *timer)
{
    struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, refresh_current_pos_timer_move);
	if (data->mode_move == ACCEL_SENSOR_MODE_ARMED)
	{
		LOG_DBG("Refreshing ref_acc_move");
		float accel = vector_length(data->last_acc_move);
		if (data->ref_acc_move.x == 0 && data->ref_acc_move.y == 0 && data->ref_acc_move.z == 0)
		{
			data->gravity = accel;
			data->ref_acc_move = data->last_acc_move;
			LOG_DBG("ref_acc_move Refreshed Init %.6f", (double)data->gravity);
		} else {
			float change = (data->gravity - accel)/data->gravity;
			if ( change < border_move && change > -border_move)
			{
				data->gravity = accel;
				data->ref_acc_move = data->last_acc_move;
				LOG_DBG("ref_acc_move Refreshed Gravity change %.6f", (double)data->gravity);
			} else {
				LOG_DBG("Move Too big difference %.6f", (double)accel);
				k_timer_start(&data->refresh_current_pos_timer_move, K_SECONDS(5), K_NO_WAIT);
				return;
			} 
		} 

	} else {
		k_timer_start(&data->refresh_current_pos_timer_move, K_SECONDS(5), K_NO_WAIT);
		return;
	}
}

static void refresh_current_pos_timer_handler_move_disarmed(struct k_timer *timer)
{
    struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, refresh_current_pos_timer_move_disarmed);
	if (data->mode_move == ACCEL_SENSOR_MODE_DISARMED)
	{
		LOG_DBG("Refreshing ref_acc_move_disarmed");
		float accel = vector_length(data->last_acc_move_disarmed);
		if (data->ref_acc_move_disarmed.x == 0 && data->ref_acc_move_disarmed.y == 0 && data->ref_acc_move_disarmed.z == 0)
		{
			data->gravity_disarmed = accel;
			data->ref_acc_move_disarmed = data->last_acc_move_disarmed;
			LOG_DBG("ref_acc_move_disarmed Refreshed Init %.6f", (double)data->gravity_disarmed);
		} else {
			float change = (data->gravity_disarmed - accel)/data->gravity_disarmed;
			if ( change < border_move && change > -border_move)
			{
				data->gravity_disarmed = accel;
				data->ref_acc_move_disarmed = data->last_acc_move_disarmed;
				LOG_DBG("ref_acc_move_disarmed Refreshed Gravity change %.6f", (double)data->gravity_disarmed);
			} else {
				LOG_DBG("Move Too big difference %.6f", (double)accel);
				k_timer_start(&data->refresh_current_pos_timer_move_disarmed, K_SECONDS(5), K_NO_WAIT);
				return;
			} 
		} 
	}
}

static void alarm_timer_handler_tilt(struct k_timer *timer)
{
	LOG_DBG("Alarm timer expired");
	struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, alarm_timer_tilt);
	if (data->mode_tilt == ACCEL_SENSOR_MODE_ALARM) {
		LOG_DBG("ACCEL_SENSOR_MODE_ARMED");
		data->mode_tilt = ACCEL_SENSOR_MODE_ARMED;
	}
}

static void alarm_timer_handler_move(struct k_timer *timer)
{
	LOG_DBG("Alarm timer expired");
	struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, alarm_timer_move);
	if (data->mode_move == ACCEL_SENSOR_MODE_ALARM) {
		LOG_DBG("ACCEL_SENSOR_MODE_ARMED_MOVE");
		data->mode_move = ACCEL_SENSOR_MODE_ARMED;
	}
}

static void increase_sensivity_timer_handler_tilt(struct k_timer *timer)
{
	struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, increase_sensivity_timer_tilt);
	int prev_warn_zone = data->current_warn_zone_tilt;
	int prev_main_zone = data->current_main_zone_tilt;
	LOG_DBG("Trying increase sensivity");
	if (data->mode_tilt == ACCEL_SENSOR_MODE_ARMED)
	{
		float pow_cos_theta = cospow2_between_vectors(data->ref_acc_tilt, data->last_acc_tilt);
		if (data->main_zone_active_tilt)
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
	struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, increase_sensivity_warn_timer_move);
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
	struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, increase_sensivity_main_timer_move);
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

static int init(const struct device *dev)
{
	const struct accel_sensor_config *cfg = dev->config;
	struct accel_sensor_data *data = dev->data;
	LOG_DBG("Initializing Accelerometer Sensor (%s)", dev->name);
	const struct device *adev = cfg->accel_dev;
	if (!device_is_ready(adev)) {
		LOG_ERR("Accelerometer device %s not ready", adev->name);
		return -ENODEV;
	}
	
	LOG_DBG("Accelerometer device: %s is ready", adev->name);

	const struct device *mma = data->accel_dev;

	sensor_attr_set(mma, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &(struct sensor_value){ .val1 = 2, .val2 = 0 });
    sensor_attr_set(mma, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &(struct sensor_value){ .val1 = 60, .val2 = 0 });

	mma8652fc_config_motion();

	init_warn_zones_tilt(dev);
	set_warn_zone_tilt(dev, 5);
	change_main_zone_tilt(dev,5);

	// struct accel_sensor_data *data = dev->data;
	int rc = 0;

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
    
	LOG_DBG("Starting periodic measurements (%d ms)", data->sampling_period_ms);
	//нахил
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
	//переміщення
	create_warn_zones_move(dev);
	create_main_zones_move(dev, data->selected_warn_zone_move);
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
	// TODO: Напевно треба перенести в макрос визначення змінної
	k_work_init_delayable(&data->dwork, adc_vbus_work_handler);
	k_work_schedule(&data->dwork, K_MSEC(data->sampling_period_ms));

	return rc;
}

#if 0
static int _attr_get(const struct device *dev, enum sensor_channel chan,
	enum sensor_attribute attr, struct sensor_value *val)
{
	// struct scd30_data *data = dev->data;
	return -ENOTSUP;
}
#endif

static int _trigger_set(const struct device *dev,
	const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler)
{
	struct accel_sensor_data *data = dev->data;
	if (trig == NULL || handler == NULL) return -EINVAL;
	switch (trig->type) {
		case ACCEL_WARN_TRIGGER:
			data->warn_handler_tilt = handler;
			data->warn_trigger_tilt = trig;
			break;
		case ACCEL_MAIN_TRIGGER:
			data->main_handler_tilt = handler;
			data->main_trigger_tilt = trig;
			break;
		case ACCEL_WARN_TRIGGER_MOVE:
			data->warn_handler_move = handler;
			data->warn_trigger_move = trig;
			break;
		case ACCEL_MAIN_TRIGGER_MOVE:
			data->main_handler_move = handler;
			data->main_trigger_move = trig;
			break;
		case ACCEL_DISARM_TRIGGER_MOVE:
			data->disarm_move_handler = handler;
			data->disarm_move_trigger = trig;
			break;
        default:
            return -ENOTSUP;
	}

	return 0;
}


static int _attr_set(const struct device *dev,
    int chan,
    int val1,
    int val2)
{
	printk("accel_attr_set called: chan=%d, val1=%d, val2=%d\n", chan, val1, val2);
	struct accel_sensor_data *data = dev->data;

	if (chan == (enum sensor_channel)ACCEL_SENSOR_MODE) {
		
		switch (data->mode_tilt) {
			case (ACCEL_SENSOR_MODE_ARMED):
				switch (val1) {
					case (ACCEL_SENSOR_MODE_ARMED):
						return 0;
					case (ACCEL_SENSOR_MODE_DISARMED):
						LOG_DBG("ACCEL_SENSOR_MODE_DISARMED");
						data->mode_tilt = val1;
						k_timer_stop(&data->alarm_timer_tilt);
						k_timer_stop(&data->refresh_current_pos_timer_tilt);
						k_timer_stop(&data->increase_sensivity_timer_tilt);
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM):
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM_STOP):
						return 0;
					default:
						return -ENOTSUP;
				}
			case (ACCEL_SENSOR_MODE_DISARMED):
				switch (val1) {
					case (ACCEL_SENSOR_MODE_ARMED):
						LOG_DBG("ACCEL_SENSOR_MODE_ARMED");
						data->current_main_zone_tilt = data->selected_main_zone_tilt;
						data->current_warn_zone_tilt = data->selected_warn_zone_tilt;
						data->in_warn_alert_tilt = false;
						data->in_main_alert_tilt = false;
						data->max_warn_alert_level_tilt = false;
						data->max_main_alert_level_tilt = false;
						data->mode_tilt = ACCEL_SENSOR_MODE_ARMED;
						data->ref_acc_tilt.x = 0;
						data->ref_acc_tilt.y = 0;
						data->ref_acc_tilt.z = 0;
						k_timer_start(&data->refresh_current_pos_timer_tilt, K_SECONDS(ARMING_DELAY_SEC), K_NO_WAIT);
						k_timer_stop(&data->increase_sensivity_timer_tilt);
						k_timer_stop(&data->alarm_timer_tilt);
						return 0;
					case (ACCEL_SENSOR_MODE_DISARMED):
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM):
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM_STOP):
						return 0;
					default:
						return -ENOTSUP;
				}
			case (ACCEL_SENSOR_MODE_ALARM):
				switch (val1) {
					case (ACCEL_SENSOR_MODE_ARMED):
						return 0;
					case (ACCEL_SENSOR_MODE_DISARMED):
						LOG_DBG("ACCEL_SENSOR_MODE_DISARMED");
						data->mode_tilt = val1;
						k_timer_stop(&data->alarm_timer_tilt);
						k_timer_stop(&data->refresh_current_pos_timer_tilt);
						k_timer_stop(&data->increase_sensivity_timer_tilt);
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM):
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM_STOP):
						k_timer_start(&data->alarm_timer_tilt, K_MSEC(STOP_ACCEL_ALARM_INTERVAL), K_NO_WAIT);
						LOG_INF("Alarm mode stoped in %d ms", STOP_ACCEL_ALARM_INTERVAL);
						return 0;
					default:
						return -ENOTSUP;
				}
			case (ACCEL_SENSOR_MODE_ALARM_STOP):
				return 0;
			default:
				return -ENOTSUP;
		}
		
    }

	if (chan == (enum sensor_channel)ACCEL_SENSOR_MODE_MOVE) {
		
		switch (data->mode_move) {
			case (ACCEL_SENSOR_MODE_ARMED):
				switch (val1) {
					case (ACCEL_SENSOR_MODE_ARMED):
						return 0;
					case (ACCEL_SENSOR_MODE_DISARMED):
						LOG_DBG("ACCEL_SENSOR_MODE_DISARMED_MOVE");
						data->mode_move = val1;
						data->sampling_period_ms = ACCEL_SENSOR_SAMPLE_TIME;
						data->samples_count_move_disarmed = 0;
						data->summary_acc_move_disarmed.x = 0;
						data->summary_acc_move_disarmed.y = 0;
						data->summary_acc_move_disarmed.z = 0;
						data->ref_acc_move_disarmed.x = 0;
						data->ref_acc_move_disarmed.y = 0;
						data->ref_acc_move_disarmed.z = 0;
						data->gravity_disarmed = 0;
						k_timer_start(&data->refresh_current_pos_timer_move_disarmed, K_SECONDS(ARMING_DELAY_SEC_DIS), K_NO_WAIT);
						k_timer_stop(&data->alarm_timer_move);
						k_timer_stop(&data->refresh_current_pos_timer_move);
						k_timer_stop(&data->increase_sensivity_warn_timer_move);
						k_timer_stop(&data->increase_sensivity_main_timer_move);
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM):
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM_STOP):
						return 0;
					default:
						return -ENOTSUP;
				}
			case (ACCEL_SENSOR_MODE_DISARMED):
				switch (val1) {
					case (ACCEL_SENSOR_MODE_ARMED):
						LOG_DBG("ACCEL_SENSOR_MODE_ARMED_MOVE");
						data->sampling_period_ms = MOVE_SENSOR_SAMPLE_TIME;
						data->current_main_zone_move = data->selected_main_zone_move;
						data->current_warn_zone_move = data->selected_warn_zone_move;
						data->max_warn_alert_level_move = false;
						data->max_main_alert_level_move = false;
						data->mode_move = ACCEL_SENSOR_MODE_ARMED;
						data->ref_acc_move.x = 0;
						data->ref_acc_move.y = 0;
						data->ref_acc_move.z = 0;
						data->samples_count_move = 0;
						data->summary_acc_move.x = 0;
						data->summary_acc_move.y = 0;
						data->summary_acc_move.z = 0;
						data->gravity = 0;
						k_timer_stop(&data->refresh_current_pos_timer_move_disarmed);
						k_timer_start(&data->refresh_current_pos_timer_move, K_SECONDS(ARMING_DELAY_SEC), K_NO_WAIT);
						k_timer_stop(&data->increase_sensivity_warn_timer_move);
						k_timer_stop(&data->increase_sensivity_main_timer_move);
						k_timer_stop(&data->alarm_timer_move);
						return 0;
					case (ACCEL_SENSOR_MODE_DISARMED):
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM):
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM_STOP):
						return 0;
					default:
						return -ENOTSUP;
				}
			case (ACCEL_SENSOR_MODE_ALARM):
				switch (val1) {
					case (ACCEL_SENSOR_MODE_ARMED):
						return 0;
					case (ACCEL_SENSOR_MODE_DISARMED):
						LOG_DBG("ACCEL_SENSOR_MODE_DISARMED_MOVE");
						data->sampling_period_ms = ACCEL_SENSOR_SAMPLE_TIME;
						data->mode_move = val1;
						data->samples_count_move_disarmed = 0;
						data->summary_acc_move_disarmed.x = 0;
						data->summary_acc_move_disarmed.y = 0;
						data->summary_acc_move_disarmed.z = 0;
						data->ref_acc_move_disarmed.x = 0;
						data->ref_acc_move_disarmed.y = 0;
						data->ref_acc_move_disarmed.z = 0;
						data->gravity_disarmed = 0;
						k_timer_start(&data->refresh_current_pos_timer_move_disarmed, K_SECONDS(ARMING_DELAY_SEC_DIS), K_NO_WAIT);
						k_timer_stop(&data->alarm_timer_move);
						k_timer_stop(&data->refresh_current_pos_timer_move);
						k_timer_stop(&data->increase_sensivity_warn_timer_move);
						k_timer_stop(&data->increase_sensivity_main_timer_move);
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM):
						return 0;
					case (ACCEL_SENSOR_MODE_ALARM_STOP):
						k_timer_start(&data->alarm_timer_move, K_MSEC(STOP_ACCEL_ALARM_INTERVAL), K_NO_WAIT);
						LOG_INF("MOVE Alarm mode stoped in %d ms", STOP_ACCEL_ALARM_INTERVAL);
						return 0;
					default:
						return -ENOTSUP;
				}
			case (ACCEL_SENSOR_MODE_ALARM_STOP):
				return 0;
			default:
				return -ENOTSUP;
		}
		
    }

	if (chan == (enum sensor_channel)ACCEL_SENSOR_CHANNEL_WARN_ZONE) {
		if (val1 == 0){
			k_timer_stop(&data->increase_sensivity_timer_tilt);
			data->warn_zone_active_tilt = false;
			data->selected_warn_zone_tilt = 9;
			data->current_warn_zone_tilt = data->selected_warn_zone_tilt;
			create_main_zones_tilt(dev, data->current_warn_zone_tilt);
			data->current_main_zone_tilt = data->selected_main_zone_tilt;
			LOG_DBG("set warn zone to %d, set main zone to %d", data->current_warn_zone_tilt, data->current_main_zone_tilt);
			data->ref_acc_tilt.x = 0;
			data->ref_acc_tilt.y = 0;
			data->ref_acc_tilt.z = 0;
			k_timer_start(&data->refresh_current_pos_timer_tilt, K_SECONDS(2), K_NO_WAIT);
			LOG_DBG("WARN_ZONE disabled");
			return 0;
		}
		val1 /= 10;
		LOG_DBG("Set warn zone to %d", 10 - val1);
		k_timer_stop(&data->increase_sensivity_timer_tilt);
		set_warn_zone_tilt(dev, 10 - val1);
		data->in_warn_alert_tilt = false;
		data->in_main_alert_tilt = false;
		data->max_warn_alert_level_tilt = false;
		data->max_main_alert_level_tilt = false;
		data->warn_zone_active_tilt = true;
		data->ref_acc_tilt.x = 0;
		data->ref_acc_tilt.y = 0;
		data->ref_acc_tilt.z = 0;
		k_timer_start(&data->refresh_current_pos_timer_tilt, K_SECONDS(2), K_NO_WAIT);
		return 0;
	}

	if (chan == (enum sensor_channel)ACCEL_SENSOR_CHANNEL_MAIN_ZONE) {
		if (val1 == 0){
			k_timer_stop(&data->increase_sensivity_timer_tilt);
			data->main_zone_active_tilt = false;
			data->current_main_zone_tilt = data->selected_main_zone_tilt;
			data->ref_acc_tilt.x = 0;
			data->ref_acc_tilt.y = 0;
			data->ref_acc_tilt.z = 0;
			k_timer_start(&data->refresh_current_pos_timer_tilt, K_SECONDS(2), K_NO_WAIT);
			LOG_DBG("MAIN_ZONE disabled");
			return 0;
		}
		val1 /= 10;
		LOG_DBG("Set main zone to %d", 10 - val1);
		k_timer_stop(&data->increase_sensivity_timer_tilt);
		change_main_zone_tilt(dev, 10 - val1);
		data->in_warn_alert_tilt = false;
		data->in_main_alert_tilt = false;
		data->max_warn_alert_level_tilt = false;
		data->max_main_alert_level_tilt = false;
		data->main_zone_active_tilt = true;
		data->ref_acc_tilt.x = 0;
		data->ref_acc_tilt.y = 0;
		data->ref_acc_tilt.z = 0;
		k_timer_start(&data->refresh_current_pos_timer_tilt, K_SECONDS(2), K_NO_WAIT);
		return 0;
	}

	///////////////////////////

	if (chan == (enum sensor_channel)ACCEL_SENSOR_CHANNEL_WARN_ZONE_MOVE) {
		if (val1 == 0){
			k_timer_stop(&data->increase_sensivity_warn_timer_move);
			k_timer_stop(&data->increase_sensivity_main_timer_move);
			data->warn_zone_active_move = false;
			data->selected_warn_zone_move = 9;
			data->current_warn_zone_move = data->selected_warn_zone_move;
			data->current_main_zone_move = data->selected_main_zone_move;
			LOG_DBG("set MOVE warn zone to %d, set main zone to %d", data->current_warn_zone_move, data->current_main_zone_move);
			data->ref_acc_move.x = 0;
			data->ref_acc_move.y = 0;
			data->ref_acc_move.z = 0;
			data->gravity = 0;
			create_main_zones_move(dev, data->current_warn_zone_move);
			k_timer_start(&data->refresh_current_pos_timer_move, K_SECONDS(2), K_NO_WAIT);
			LOG_DBG("WARN_ZONE_MOVE disabled");
			return 0;
		}
		val1 /= 10;
		LOG_DBG("Set MOVE warn zone to %d", 10 - val1);
		k_timer_stop(&data->increase_sensivity_main_timer_move);
		k_timer_stop(&data->increase_sensivity_warn_timer_move);
		data->selected_warn_zone_move = 10 - val1;
		data->current_warn_zone_move = data->selected_warn_zone_move;
		create_main_zones_move(dev, data->selected_warn_zone_move);
		data->current_main_zone_move = data->selected_main_zone_move;
		data->max_warn_alert_level_move = false;
		data->max_main_alert_level_move = false;
		data->warn_zone_active_move = true;
		data->ref_acc_move.x = 0;
		data->ref_acc_move.y = 0;
		data->ref_acc_move.z = 0;
		data->gravity = 0;
		k_timer_start(&data->refresh_current_pos_timer_move, K_SECONDS(2), K_NO_WAIT);
		return 0;
	}

	if (chan == (enum sensor_channel)ACCEL_SENSOR_CHANNEL_MAIN_ZONE_MOVE) {
		if (val1 == 0){
			k_timer_stop(&data->increase_sensivity_warn_timer_move);
			k_timer_stop(&data->increase_sensivity_main_timer_move);
			data->main_zone_active_move = false;
			data->current_main_zone_move = data->selected_main_zone_move;
			data->current_warn_zone_move = data->selected_warn_zone_move;
			data->ref_acc_move.x = 0;
			data->ref_acc_move.y = 0;
			data->ref_acc_move.z = 0;
			data->gravity = 0;
			k_timer_start(&data->refresh_current_pos_timer_move, K_SECONDS(2), K_NO_WAIT);
			LOG_DBG("MAIN_ZONE_MOVE disabled");
			return 0;
		}
		val1 /= 10;
		LOG_DBG("Set MOVE main zone to %d", 10 - val1);
		k_timer_stop(&data->increase_sensivity_warn_timer_move);
		k_timer_stop(&data->increase_sensivity_main_timer_move);
		data->selected_main_zone_move = 10 - val1;
		data->current_main_zone_move = data->selected_main_zone_move;
		data->current_warn_zone_move = data->selected_warn_zone_move;
		data->max_warn_alert_level_move = false;
		data->max_main_alert_level_move = false;
		data->main_zone_active_move = true;
		data->ref_acc_move.x = 0;
		data->ref_acc_move.y = 0;
		data->ref_acc_move.z = 0;
		data->gravity = 0;
		k_timer_start(&data->refresh_current_pos_timer_move, K_SECONDS(2), K_NO_WAIT);
		return 0;
	}

	return -ENOTSUP;
}


static const struct accel_sensor_driver_api driver_api = {
	.set_current_position_as_reference = _save_current_positoin_as_reference,
	.attr_set = _attr_set,
	.trigger_set = _trigger_set,
	// .sample_fetch = scd30_sample_fetch,
	// .channel_get = scd30_channel_get,
	// .attr_get = _attr_get,
	// .attr_set = _attr_set,
};

#define ACCEL_SENSOR_DEFINE(inst)									    \
	static struct accel_sensor_data data_##inst = {						\
		.sampling_period_ms = DT_INST_PROP(inst, sampling_period_ms),		\
		.accel_dev = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))), 			\
	};											                        \
	static const struct accel_sensor_config config_##inst = {			\
		.accel_dev = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))), 			\
	};											                        \
												                        \
	DEVICE_DT_INST_DEFINE(inst, init, NULL, &data_##inst, &config_##inst,	\
			      POST_KERNEL, CONFIG_ACCEL_SENSOR_INIT_PRIORITY, &driver_api);

// CONFIG_KERNEL_INIT_PRIORITY_DEVICE

DT_INST_FOREACH_STATUS_OKAY(ACCEL_SENSOR_DEFINE);
