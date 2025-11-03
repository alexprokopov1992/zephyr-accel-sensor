/* This file contains the detection logic copied from the original driver */
/* It should be included at the end of lis2dw12-sensor.c */
#include "lis2dw12-sensor.h"


static void coarsering_tilt(struct lis2dw12_data *data, int level)
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

static void coarsering_move(struct lis2dw12_data *data, int level)
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

static bool both_mode_disarmed(struct lis2dw12_data *data)
{
	if (data->mode_tilt == ACCEL_SENSOR_MODE_DISARMED && data->mode_move == ACCEL_SENSOR_MODE_DISARMED)
	{
		return true;
	}
	return false;
}

static void init_warn_zones_tilt(const struct device *dev)
{
	struct lis2dw12_data *data = dev->data;
	for (int i = 0; i < 10; i++) {
		float angle = warn_zone_start_angle + warn_zone_step_angle * i;
		data->warn_zone_cos_pow2[i] = cosf(angle * M_PIf / 180.0f);
		data->warn_zone_cos_pow2[i] *= data->warn_zone_cos_pow2[i];
		LOG_DBG("warn_zone_cos_pow2[%d]: %d.%06d", i, 
				(int)data->warn_zone_cos_pow2[i], 
				(int)((data->warn_zone_cos_pow2[i] - (int)data->warn_zone_cos_pow2[i]) * 1000000));
	}
}

static void create_main_zones_tilt(const struct device *dev, int warn_level)
{
	struct lis2dw12_data *data = dev->data;
	float start_angle = warn_zone_start_angle + warn_zone_step_angle * warn_level;
	float step_angle = (max_angle - start_angle) / 9.0f;
	
	for (int i = 0; i < 10; i++) {
		float angle = start_angle + step_angle * i;
		data->main_zone_cos_pow2[i] = cosf(angle * M_PIf / 180.0f);
		data->main_zone_cos_pow2[i] *= data->main_zone_cos_pow2[i];
		LOG_DBG("main_zone_cos_pow2[%d]: %d.%06d", i,
				(int)data->main_zone_cos_pow2[i],
				(int)((data->main_zone_cos_pow2[i] - (int)data->main_zone_cos_pow2[i]) * 1000000));
	}
}

static void set_warn_zone_tilt(const struct device *dev, int level)
{
	struct lis2dw12_data *data = dev->data;
	data->selected_warn_zone_tilt = level;
	data->current_warn_zone_tilt = level;
	create_main_zones_tilt(dev, level);
	data->selected_main_zone_tilt = level;
	data->current_main_zone_tilt = level;
	LOG_DBG("Set warn zone to %d, main zone to %d", level, level);
}

static void change_main_zone_tilt(const struct device *dev, int level)
{
	struct lis2dw12_data *data = dev->data;
	data->selected_main_zone_tilt = level;
	data->current_main_zone_tilt = level;
	LOG_DBG("Changed main zone to %d", level);
}

static void create_warn_zones_move(const struct device *dev)
{
	struct lis2dw12_data *data = dev->data;
	for (int i = 0; i < 10; i++) {
		data->warn_zone_move[i] = border_move + warn_zone_accel_mult * i;
		LOG_DBG("warn_zone_move[%d]: %d.%06d", i,
				(int)data->warn_zone_move[i],
				(int)((data->warn_zone_move[i] - (int)data->warn_zone_move[i]) * 1000000));
	}
}

static void create_main_zones_move(const struct device *dev, int warn_level)
{
	struct lis2dw12_data *data = dev->data;
	float start_mult = warn_zone_accel_mult * warn_level;
	float step_mult = (main_zone_max_mult - start_mult) / 9.0f;
	
	for (int i = 0; i < 10; i++) {
		data->main_zone_move[i] = border_move + start_mult + step_mult * i;
		LOG_DBG("main_zone_move[%d]: %d.%06d", i,
				(int)data->main_zone_move[i],
				(int)((data->main_zone_move[i] - (int)data->main_zone_move[i]) * 1000000));
	}
}

