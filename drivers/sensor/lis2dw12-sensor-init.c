/* Initialization and API functions for LIS2DW12 driver */

/* Work handler for periodic data processing */
static void accel_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct lis2dw12_data *data = CONTAINER_OF(dwork, struct lis2dw12_data, dwork);
	const struct device *dev = data->dev;
	_Vector3 current_acc;
	int ret;

	/* Wait for data ready from interrupt */
	ret = k_sem_take(&data->data_ready_sem, K_MSEC(data->sampling_period_ms + 100));
	if (ret != 0) {
		LOG_WRN("Timeout waiting for data ready");
		goto schedule_next;
	}

	/* Read accelerometer data */
	ret = get_accel_values(dev, &current_acc);
	if (ret < 0) {
		LOG_ERR("Failed to read accelerometer data");
		goto schedule_next;
	}

	data->last_acc_tilt = current_acc;
	data->last_acc_move = current_acc;

	/* Process tilt detection */
	if (data->mode_tilt == ACCEL_SENSOR_MODE_ARMED) {
		if (data->ref_acc_tilt.x == 0 && data->ref_acc_tilt.y == 0 && data->ref_acc_tilt.z == 0) {
			data->ref_acc_tilt = current_acc;
			LOG_DBG("Set reference position TILT");
		} else {
			float pow_cos_theta = cospow2_between_vectors(data->ref_acc_tilt, current_acc);
			
			/* Check warn zone */
			if (data->warn_zone_active_tilt && 
			    pow_cos_theta < data->warn_zone_cos_pow2[data->current_warn_zone_tilt] &&
			    !data->in_warn_alert_tilt) {
				int64_t current_time = k_uptime_get();
				if ((current_time - data->last_trigger_time_warn_tilt) > MIN_WARN_INTERVAL) {
					data->in_warn_alert_tilt = true;
					data->last_trigger_time_warn_tilt = current_time;
					LOG_WRN("WARN_ZONE TILT alert! Level: %d", data->current_warn_zone_tilt);
					k_timer_start(&data->increase_sensivity_timer_tilt, 
					             K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
					if (data->warn_handler_tilt && data->warn_trigger_tilt) {
						data->warn_handler_tilt(dev, data->warn_trigger_tilt);
					}
					if (!data->max_warn_alert_level_tilt) {
						coarsering_tilt(data, 0);
					}
				}
			}
			
			/* Check main zone */
			if (data->main_zone_active_tilt && 
			    pow_cos_theta < data->main_zone_cos_pow2[data->current_main_zone_tilt] &&
			    !data->in_main_alert_tilt) {
				int64_t current_time = k_uptime_get();
				if ((current_time - data->last_trigger_time_main_tilt) > MIN_WARN_INTERVAL) {
					data->in_main_alert_tilt = true;
					data->last_trigger_time_main_tilt = current_time;
					data->mode_tilt = ACCEL_SENSOR_MODE_ALARM;
					LOG_ERR("MAIN_ZONE TILT ALARM! Level: %d", data->current_main_zone_tilt);
					k_timer_start(&data->increase_sensivity_timer_tilt, 
					             K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
					if (data->main_handler_tilt && data->main_trigger_tilt) {
						data->main_handler_tilt(dev, data->main_trigger_tilt);
					}
					if (!data->max_main_alert_level_tilt) {
						coarsering_tilt(data, 1);
					}
				}
			}
		}
	}

	/* Process movement detection */
	if (data->mode_move == ACCEL_SENSOR_MODE_ARMED) {
		data->samples_count_move++;
		data->summary_acc_move.x += current_acc.x;
		data->summary_acc_move.y += current_acc.y;
		data->summary_acc_move.z += current_acc.z;

		if (data->samples_count_move >= MOVE_SENSOR_SAMPLE_COUNT) {
			_Vector3 avg_acc;
			avg_acc.x = data->summary_acc_move.x / data->samples_count_move;
			avg_acc.y = data->summary_acc_move.y / data->samples_count_move;
			avg_acc.z = data->summary_acc_move.z / data->samples_count_move;

			if (data->gravity == 0) {
				data->gravity = vector_length(avg_acc);
				data->ref_acc_move = avg_acc;
				LOG_DBG("Set reference position MOVE, gravity: %d.%03d", 
				        (int)data->gravity, (int)((data->gravity - (int)data->gravity) * 1000));
			} else {
				float current_gravity = vector_length(avg_acc);
				float delta = fabsf(current_gravity - data->gravity);
				
				/* Check warn zone */
				if (data->warn_zone_active_move && 
				    delta > data->warn_zone_move[data->current_warn_zone_move]) {
					int64_t current_time = k_uptime_get();
					if ((current_time - data->last_trigger_time_warn_move) > MIN_WARN_INTERVAL) {
						data->last_trigger_time_warn_move = current_time;
						LOG_WRN("WARN_ZONE MOVE alert! Level: %d, delta: %d.%06d", 
						        data->current_warn_zone_move,
						        (int)delta, (int)((delta - (int)delta) * 1000000));
						k_timer_start(&data->increase_sensivity_warn_timer_move, 
						             K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
						if (data->warn_handler_move && data->warn_trigger_move) {
							data->warn_handler_move(dev, data->warn_trigger_move);
						}
						if (!data->max_warn_alert_level_move) {
							coarsering_move(data, 0);
						}
					}
				}
				
				/* Check main zone */
				if (data->main_zone_active_move && 
				    delta > data->main_zone_move[data->current_main_zone_move]) {
					int64_t current_time = k_uptime_get();
					if ((current_time - data->last_trigger_time_main_move) > MIN_WARN_INTERVAL) {
						data->last_trigger_time_main_move = current_time;
						data->mode_move = ACCEL_SENSOR_MODE_ALARM;
						LOG_ERR("MAIN_ZONE MOVE ALARM! Level: %d, delta: %d.%06d", 
						        data->current_main_zone_move,
						        (int)delta, (int)((delta - (int)delta) * 1000000));
						k_timer_start(&data->increase_sensivity_main_timer_move, 
						             K_SECONDS(INCREASE_SENSIVITY_TIME), K_NO_WAIT);
						if (data->main_handler_move && data->main_trigger_move) {
							data->main_handler_move(dev, data->main_trigger_move);
						}
						if (!data->max_main_alert_level_move) {
							coarsering_move(data, 1);
						}
					}
				}
			}

			/* Reset accumulation */
			data->samples_count_move = 0;
			data->summary_acc_move.x = 0;
			data->summary_acc_move.y = 0;
			data->summary_acc_move.z = 0;
		}
	} else if (data->mode_move == ACCEL_SENSOR_MODE_DISARMED) {
		/* Accumulate data for calibration */
		data->samples_count_move_disarmed++;
		data->summary_acc_move_disarmed.x += current_acc.x;
		data->summary_acc_move_disarmed.y += current_acc.y;
		data->summary_acc_move_disarmed.z += current_acc.z;
	}

schedule_next:
	/* Schedule next reading */
	k_work_schedule(&data->dwork, K_MSEC(data->sampling_period_ms));
}

static int _save_current_positoin_as_reference(const struct device *dev)
{
	struct lis2dw12_data *data = dev->data;
	
	/* Reset tilt reference */
	data->ref_acc_tilt.x = 0;
	data->ref_acc_tilt.y = 0;
	data->ref_acc_tilt.z = 0;
	
	/* Reset move reference */
	data->samples_count_move = 0;
	data->summary_acc_move.x = 0;
	data->summary_acc_move.y = 0;
	data->summary_acc_move.z = 0;
	data->ref_acc_move.x = 0;
	data->ref_acc_move.y = 0;
	data->ref_acc_move.z = 0;
	data->gravity = 0;
	
	LOG_INF("Current position saved as reference");
	return 0;
}

static int _trigger_set(const struct device *dev,
		const struct sensor_trigger *trig,
		sensor_trigger_handler_t handler)
{
	struct lis2dw12_data *data = dev->data;

	switch (trig->type) {
	case ACCEL_WARN_TRIGGER:
		data->warn_handler_tilt = handler;
		data->warn_trigger_tilt = trig;
		LOG_DBG("Set WARN_TRIGGER handler");
		return 0;
	case ACCEL_MAIN_TRIGGER:
		data->main_handler_tilt = handler;
		data->main_trigger_tilt = trig;
		LOG_DBG("Set MAIN_TRIGGER handler");
		return 0;
	case ACCEL_WARN_TRIGGER_MOVE:
		data->warn_handler_move = handler;
		data->warn_trigger_move = trig;
		LOG_DBG("Set WARN_TRIGGER_MOVE handler");
		return 0;
	case ACCEL_MAIN_TRIGGER_MOVE:
		data->main_handler_move = handler;
		data->main_trigger_move = trig;
		LOG_DBG("Set MAIN_TRIGGER_MOVE handler");
		return 0;
	case ACCEL_DISARM_TRIGGER_MOVE:
		data->disarm_move_handler = handler;
		data->disarm_move_trigger = trig;
		LOG_DBG("Set DISARM_TRIGGER_MOVE handler");
		return 0;
	default:
		LOG_ERR("Unsupported trigger type");
		return -ENOTSUP;
	}
}

static int _attr_set(const struct device *dev, int channel, int val1, int val2)
{
	struct lis2dw12_data *data = dev->data;

	if (channel == ACCEL_SENSOR_MODE) {
		/* Handle tilt mode changes */
		switch (data->mode_tilt) {
		case ACCEL_SENSOR_MODE_DISARMED:
			if (val1 == ACCEL_SENSOR_MODE_ARMED) {
				LOG_DBG("TILT: DISARMED -> ARMED");
				data->mode_tilt = val1;
				data->sampling_period_ms = ACCEL_SENSOR_SAMPLE_TIME;
				data->ref_acc_tilt.x = 0;
				data->ref_acc_tilt.y = 0;
				data->ref_acc_tilt.z = 0;
				k_timer_start(&data->refresh_current_pos_timer_tilt, 
				             K_SECONDS(REFRESH_POS_TIME), K_NO_WAIT);
				return 0;
			}
			break;
		case ACCEL_SENSOR_MODE_ARMED:
			if (val1 == ACCEL_SENSOR_MODE_DISARMED) {
				LOG_DBG("TILT: ARMED -> DISARMED");
				data->mode_tilt = val1;
				k_timer_stop(&data->refresh_current_pos_timer_tilt);
				k_timer_stop(&data->increase_sensivity_timer_tilt);
				return 0;
			}
			break;
		case ACCEL_SENSOR_MODE_ALARM:
			if (val1 == ACCEL_SENSOR_MODE_ALARM_STOP) {
				k_timer_start(&data->alarm_timer_tilt, 
				             K_MSEC(STOP_ACCEL_ALARM_INTERVAL), K_NO_WAIT);
				LOG_INF("TILT Alarm mode will stop in %d ms", STOP_ACCEL_ALARM_INTERVAL);
				return 0;
			}
			break;
		}
	}

	if (channel == ACCEL_SENSOR_MODE_MOVE) {
		/* Handle move mode changes */
		switch (data->mode_move) {
		case ACCEL_SENSOR_MODE_DISARMED:
			if (val1 == ACCEL_SENSOR_MODE_ARMED) {
				LOG_DBG("MOVE: DISARMED -> ARMED");
				data->mode_move = val1;
				data->sampling_period_ms = MOVE_SENSOR_SAMPLE_TIME;
				data->samples_count_move = 0;
				data->summary_acc_move.x = 0;
				data->summary_acc_move.y = 0;
				data->summary_acc_move.z = 0;
				data->ref_acc_move.x = 0;
				data->ref_acc_move.y = 0;
				data->ref_acc_move.z = 0;
				data->gravity = 0;
				k_timer_start(&data->refresh_current_pos_timer_move, 
				             K_SECONDS(REFRESH_POS_TIME_MOVE), K_NO_WAIT);
				return 0;
			}
			break;
		case ACCEL_SENSOR_MODE_ARMED:
			if (val1 == ACCEL_SENSOR_MODE_DISARMED) {
				LOG_DBG("MOVE: ARMED -> DISARMED");
				data->sampling_period_ms = ACCEL_SENSOR_SAMPLE_TIME;
				data->mode_move = val1;
				data->samples_count_move_disarmed = 0;
				data->summary_acc_move_disarmed.x = 0;
				data->summary_acc_move_disarmed.y = 0;
				data->summary_acc_move_disarmed.z = 0;
				k_timer_start(&data->refresh_current_pos_timer_move_disarmed, 
				             K_SECONDS(ARMING_DELAY_SEC_DIS), K_NO_WAIT);
				k_timer_stop(&data->refresh_current_pos_timer_move);
				k_timer_stop(&data->increase_sensivity_warn_timer_move);
				k_timer_stop(&data->increase_sensivity_main_timer_move);
				return 0;
			}
			break;
		case ACCEL_SENSOR_MODE_ALARM:
			if (val1 == ACCEL_SENSOR_MODE_ALARM_STOP) {
				k_timer_start(&data->alarm_timer_move, 
				             K_MSEC(STOP_ACCEL_ALARM_INTERVAL), K_NO_WAIT);
				LOG_INF("MOVE Alarm mode will stop in %d ms", STOP_ACCEL_ALARM_INTERVAL);
				return 0;
			}
			break;
		}
	}

	/* Zone configuration */
	if (channel == ACCEL_SENSOR_CHANNEL_WARN_ZONE) {
		if (val1 == 0) {
			k_timer_stop(&data->increase_sensivity_timer_tilt);
			data->warn_zone_active_tilt = false;
			LOG_DBG("WARN_ZONE disabled");
		} else {
			val1 /= 10;
			LOG_DBG("Set warn zone to %d", 10 - val1);
			k_timer_stop(&data->increase_sensivity_timer_tilt);
			set_warn_zone_tilt(dev, 10 - val1);
			data->warn_zone_active_tilt = true;
			data->ref_acc_tilt.x = 0;
			data->ref_acc_tilt.y = 0;
			data->ref_acc_tilt.z = 0;
			k_timer_start(&data->refresh_current_pos_timer_tilt, K_SECONDS(2), K_NO_WAIT);
		}
		return 0;
	}

	if (channel == ACCEL_SENSOR_CHANNEL_MAIN_ZONE) {
		if (val1 == 0) {
			k_timer_stop(&data->increase_sensivity_timer_tilt);
			data->main_zone_active_tilt = false;
			LOG_DBG("MAIN_ZONE disabled");
		} else {
			val1 /= 10;
			LOG_DBG("Set main zone to %d", 10 - val1);
			k_timer_stop(&data->increase_sensivity_timer_tilt);
			change_main_zone_tilt(dev, 10 - val1);
			data->main_zone_active_tilt = true;
			data->ref_acc_tilt.x = 0;
			data->ref_acc_tilt.y = 0;
			data->ref_acc_tilt.z = 0;
			k_timer_start(&data->refresh_current_pos_timer_tilt, K_SECONDS(2), K_NO_WAIT);
		}
		return 0;
	}

	return -ENOTSUP;
}

static const struct accel_sensor_driver_api driver_api = {
	.set_current_position_as_reference = _save_current_positoin_as_reference,
	.attr_set = _attr_set,
	.trigger_set = _trigger_set,
};
