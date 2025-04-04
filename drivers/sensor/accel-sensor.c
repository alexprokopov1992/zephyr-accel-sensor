#define DT_DRV_COMPAT zephyr_accel_sensor

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

#include "accel-sensor.h"
#include <math.h>

LOG_MODULE_REGISTER(accel_sensor, LOG_LEVEL_DBG);

#if !defined(M_PIf)
#define M_PIf 3.1415927f
#endif

#define REFRESH_POS_TIME 3600
#define INCREASE_SENSIVITY_TIME 10

static float warn_zone_start_angle = 1.0;
static float warn_zone_step_angle = 2.0/9.0;
static float max_angle = 10.00;
static const float cos_pow_0_5  = 0.999961923;

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


static void coarsering(struct accel_sensor_data *data, int level)
{
	if (level == 0)
	{
		if (data->current_warn_zone == 9){
			data->max_warn_alert_level = true;
			LOG_DBG("Max warn level reached %d", data->current_warn_zone);
		} else if (data->warn_zone_cos_pow2[data->current_warn_zone+1] < data->main_zone_cos_pow2[data->selected_main_zone])
		{
			data->max_warn_alert_level = true;
			LOG_DBG("Max warn level reached %d", data->current_warn_zone);
		} else {
			data->current_warn_zone++;
			LOG_DBG("Coarsering WARN_ZONE to %d", data->current_warn_zone);
		}
	} else {
		if (data->current_main_zone == 9){
			data->max_warn_alert_level = true;
			data->max_main_alert_level = true;
			while (data->warn_zone_cos_pow2[data->current_warn_zone] > data->main_zone_cos_pow2[data->selected_main_zone])
			{
				if (data->current_warn_zone == 9) break;
				data->current_warn_zone++;
			}
			if (data->warn_zone_cos_pow2[data->current_warn_zone] <= data->main_zone_cos_pow2[data->selected_main_zone] && data->current_warn_zone > 0) data->current_warn_zone--;
			LOG_DBG("Coarsering WARN_ZONE to %d", data->current_warn_zone);
			LOG_DBG("Max main level reached %d", data->current_main_zone);
		} else {
			data->current_main_zone += 1;
			data->max_warn_alert_level = true;
			while (data->warn_zone_cos_pow2[data->current_warn_zone] > data->main_zone_cos_pow2[data->selected_main_zone])
			{
				if (data->current_warn_zone == 9) break;
				data->current_warn_zone++;
			}
			if (data->warn_zone_cos_pow2[data->current_warn_zone] <= data->main_zone_cos_pow2[data->selected_main_zone] && data->current_warn_zone > 0) data->current_warn_zone--;
			LOG_DBG("Max warn level reached %d", data->current_warn_zone);
			LOG_DBG("Coarsering MAIN_ZONE to %d", data->current_main_zone);
		}
	}
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
    data->last_acc = current_acc;
    // Обчислення загального кута нахилу
    // float theta = angle_between_vectors(data->ref_acc, current_acc);
    // printk("Загальний кут нахилу: %.3f градусів\n", theta * (180.0f / M_PI));

	if (data->mode != ACCEL_SENSOR_MODE_ARMED) {
		k_work_schedule(&data->dwork, K_MSEC(data->sampling_period_ms));
		return;
	}

	float pow_cos_theta = cospow2_between_vectors(data->ref_acc, current_acc);
	if (pow_cos_theta == 0) {
		k_work_schedule(&data->dwork, K_MSEC(data->sampling_period_ms));
		return;
	}
	if (pow_cos_theta < data->main_zone_cos_pow2[data->current_main_zone])
	{
		if (!data->max_main_alert_level){
			data->main_handler(dev, data->main_trigger);
			data->in_warn_alert = true;
			data->in_main_alert = true;
			coarsering(data, 1);
			LOG_DBG("MAIN TRIGGER");
			k_timer_start(&data->increase_sensivity_timer, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
		}
		
	} else if (pow_cos_theta < data->warn_zone_cos_pow2[data->current_warn_zone]){
		if (!data->max_warn_alert_level) {
			data->in_warn_alert = true;
			data->warn_handler(dev, data->warn_trigger);
			coarsering(data, 0);
			LOG_DBG("WARN TRIGGER");
			k_timer_start(&data->increase_sensivity_timer, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
		}	
	}

	LOG_DBG(
		"Current values: X:%.4f Y:%.4f Z:%.4f"
		" pow cos theta: %.8f"
		, (double)ax, (double)ay, (double)az
		, (double)(pow_cos_theta)
	);

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

static void init_warn_zones(const struct device *dev)
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

static void create_main_zones(const struct device *dev, int warn_zone)
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

static void set_warn_zone(const struct device *dev, int zone)
{
	struct accel_sensor_data *data = dev->data;
	data->selected_warn_zone = zone;
	data->current_warn_zone = zone;
	create_main_zones(dev, zone);
	data->current_main_zone = 0;
	data->selected_main_zone = 0;
}

static void change_main_zone(const struct device *dev, int zone)
{
	struct accel_sensor_data *data = dev->data;
	data->current_main_zone = zone;
	data->selected_main_zone = zone;
	data->current_warn_zone = data->selected_warn_zone;
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
	data->ref_acc = data->last_acc;
	return 0;
}

static void refresh_current_pos_timer_handler(struct k_timer *timer)
{
	LOG_DBG("Refreshing ref_acc");
    struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, refresh_current_pos_timer);
	if (data->mode == ACCEL_SENSOR_MODE_ARMED)
	{
		if (!data->in_warn_alert && !data->in_main_alert)
		{
			float pow_cos_theta = cospow2_between_vectors(data->ref_acc, data->last_acc);
			if (pow_cos_theta >  cos_pow_0_5 || pow_cos_theta == 0)
			{
				data->ref_acc = data->last_acc;
				LOG_DBG("ref_acc Refreshed");
			}  
			LOG_DBG("cosinuses: %.8f %.8f", (double)pow_cos_theta, (double)cos_pow_0_5); 
		}
	} else {
		k_timer_start(&data->refresh_current_pos_timer, K_SECONDS(5), K_NO_WAIT);
		return;
	}
	k_timer_start(&data->refresh_current_pos_timer, K_SECONDS(REFRESH_POS_TIME), K_NO_WAIT);
}

static void alarm_timer_handler(struct k_timer *timer)
{
	LOG_DBG("Alarm timer expired");
	struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, alarm_timer);
	if (data->mode == ACCEL_SENSOR_MODE_ALARM) {
		LOG_DBG("ACCEL_SENSOR_MODE_ARMED");
		data->mode = ACCEL_SENSOR_MODE_ARMED;
	}
}

static void increase_sensivity_timer_handler(struct k_timer *timer)
{
	struct accel_sensor_data *data = CONTAINER_OF(timer, struct accel_sensor_data, increase_sensivity_timer);
	int prev_warn_zone = data->current_warn_zone;
	int prev_main_zone = data->current_main_zone;
	LOG_DBG("Trying increase sensivity");
	if (data->mode == ACCEL_SENSOR_MODE_ARMED)
	{
		float pow_cos_theta = cospow2_between_vectors(data->ref_acc, data->last_acc);
		while (data->current_main_zone > data->selected_main_zone)
		{
			if (data->main_zone_cos_pow2[data->current_main_zone] < pow_cos_theta) {
				data->current_main_zone--;
			} else {
				data->current_main_zone++;
				break;
			}
		}
		if (data->current_main_zone == data->selected_main_zone)
		{
			while (data->current_warn_zone > data->selected_warn_zone)
			{
				if (data->warn_zone_cos_pow2[data->current_warn_zone] < pow_cos_theta) {
					data->current_warn_zone--;
				} else {
					data->current_warn_zone++;
					break;
				}
			}
		}
		
	} else {
		k_timer_start(&data->increase_sensivity_timer, K_SECONDS(30), K_NO_WAIT);
		return;
	}

	if (data->current_main_zone != prev_main_zone){
		LOG_DBG("Main zone changed from %d to %d", prev_main_zone, data->current_main_zone);
		data->max_main_alert_level = false;
	}

	if (data->current_warn_zone != prev_warn_zone){
		LOG_DBG("Warn zone changed from %d to %d", prev_warn_zone, data->current_warn_zone);
		data->max_warn_alert_level = false;
	} 

	if (data->current_warn_zone == data->selected_warn_zone) {
		data->in_warn_alert = false;
	}

	if (data->current_main_zone == data->selected_main_zone) {
		data->in_main_alert = false;
	}

	if (data->in_warn_alert || data->in_main_alert) {
		k_timer_start(&data->increase_sensivity_timer, K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
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
    sensor_attr_set(mma, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &(struct sensor_value){ .val1 = 50, .val2 = 0 });

	init_warn_zones(dev);
	set_warn_zone(dev, 0);
	change_main_zone(dev,0);

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
	data->in_warn_alert = false;
	data->in_main_alert = false;
	data->max_warn_alert_level = false;
	data->max_main_alert_level = false;
	data->mode = ACCEL_SENSOR_MODE_DISARMED;
	k_timer_init(&data->refresh_current_pos_timer, refresh_current_pos_timer_handler, NULL);
	k_timer_init(&data->increase_sensivity_timer, increase_sensivity_timer_handler, NULL);
	k_timer_init(&data->alarm_timer, alarm_timer_handler, NULL);
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
			data->warn_handler = handler;
			data->warn_trigger = trig;
			break;
		case ACCEL_MAIN_TRIGGER:
			data->main_handler = handler;
			data->main_trigger = trig;
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
	printk("attr_set called: chan=%d, val1=%d, val2=%d\n", chan, val1, val2);
	struct accel_sensor_data *data = dev->data;

	if (chan == (enum sensor_channel)ACCEL_SENSOR_MODE) {
		if (val1 == data->mode) return 0;

        switch(val1){
			case (ACCEL_SENSOR_MODE_ARMED):
				LOG_DBG("ACCEL_SENSOR_MODE_ARMED");
				data->current_main_zone = data->selected_main_zone;
				data->current_warn_zone = data->selected_warn_zone;
				data->in_warn_alert = false;
				data->in_main_alert = false;
				data->max_warn_alert_level = false;
				data->max_main_alert_level = false;
				data->mode = ACCEL_SENSOR_MODE_ARMED;
				k_timer_start(&data->refresh_current_pos_timer, K_MSEC(500), K_NO_WAIT);
				k_timer_stop(&data->increase_sensivity_timer);
				k_timer_stop(&data->alarm_timer);
				break;
			case (ACCEL_SENSOR_MODE_DISARMED):
				LOG_DBG("ACCEL_SENSOR_MODE_DISARMED");
			    data->mode = val1;
				k_timer_stop(&data->alarm_timer);
				k_timer_stop(&data->refresh_current_pos_timer);
				k_timer_stop(&data->increase_sensivity_timer);
				break;
			case (ACCEL_SENSOR_MODE_TURN_OFF):
				LOG_DBG("ACCEL_SENSOR_MODE_TURN_OFF");
				data->mode = val1;
				k_timer_stop(&data->alarm_timer);
				k_timer_stop(&data->refresh_current_pos_timer);
				k_timer_stop(&data->increase_sensivity_timer);
				break;
			case (ACCEL_SENSOR_MODE_ALARM):
				if (data->mode == ACCEL_SENSOR_MODE_ARMED) {
					k_timer_start(&data->alarm_timer, K_MSEC(val2), K_NO_WAIT);
					LOG_DBG("ACCEL_SENSOR_MODE_ALARM");
					data->mode = ACCEL_SENSOR_MODE_ALARM;
				}
				break;
			case (ACCEL_SENSOR_MODE_ALARM_STOP):
				if (data->mode == ACCEL_SENSOR_MODE_ALARM)
				{
					k_timer_stop(&data->alarm_timer);
					LOG_DBG("ACCEL_SENSOR_MODE_ALARM_STOP");
					data->mode = ACCEL_SENSOR_MODE_ARMED;
				}
				break;
			default:
				break;
		}
        return 0;
    }

	if (chan == (enum sensor_channel)ACCEL_SENSOR_CHANNEL_WARN_ZONE) {
		LOG_DBG("Set warn zone to %d", 10 - val1);
		set_warn_zone(dev, 10 - val1);
		data->in_warn_alert = false;
		data->in_main_alert = false;
		data->max_warn_alert_level = false;
		data->max_main_alert_level = false;
		k_timer_start(&data->refresh_current_pos_timer, K_MSEC(100), K_NO_WAIT);
		k_timer_stop(&data->increase_sensivity_timer);
	}

	if (chan == (enum sensor_channel)ACCEL_SENSOR_CHANNEL_MAIN_ZONE) {
		LOG_DBG("Set main zone to %d", 10 - val1);
		change_main_zone(dev, 10 - val1);
		data->in_warn_alert = false;
		data->in_main_alert = false;
		data->max_warn_alert_level = false;
		data->max_main_alert_level = false;
		k_timer_start(&data->refresh_current_pos_timer, K_MSEC(100), K_NO_WAIT);
		k_timer_stop(&data->increase_sensivity_timer);

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
