#include "vesc.h"
#include "can.h"
#include "datatypes.h"
#include "math.h"
#include "timer.h"

#define BUFFER_LENGTH	5

uint8_t power_buffer[BUFFER_LENGTH];
uint8_t power_ptr;
uint16_t power_tmp;

const float wheel_diam = 0.1;	// meters
const float motor_poles = 20.0;
// const float gear_ratio = 78.0 / 11.0;
const float gear_ratio = 1;
float last_power_in;
float last_power_motor;

volatile vesc_calc_data calc_data = { -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1 };


void vesc_calculate_can_data() {
	static timer_ticks_t consumption_ticks = 0;

	calc_data.speed_meter_per_sec = wheel_diam * M_PI * (can_values.rpm / (motor_poles / 2.0) / gear_ratio / 60.0);
	calc_data.speed_kmh = calc_data.speed_meter_per_sec * 3.6;
	calc_data.power_in = can_values.v_in * can_values.current_in;
	calc_data.power_motor = can_values.v_in * can_values.current_motor;
	calc_data.distance_meter = can_values.tachometer_abs * wheel_diam * M_PI / gear_ratio / motor_poles / 3.0;

	if(systicks - consumption_ticks > 50) {

		power_buffer[power_ptr++] = fabsf(calc_data.power_in);
		power_ptr %= BUFFER_LENGTH;
		for(uint8_t i = 0; i < BUFFER_LENGTH-1; i++) {
			power_tmp += power_buffer[i];
		}
		power_tmp /= BUFFER_LENGTH;
		calc_data.power_in_filtered = power_tmp & 0xff;

		consumption_ticks = systicks;
	}


}

