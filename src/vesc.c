#include "vesc.h"
#include "can.h"
#include "datatypes.h"
#include "math.h"
#include "timer.h"


const float wheel_diam = 0.1;	// meters
const float motor_poles = 20.0;
// const float gear_ratio = 78.0 / 11.0;
const float gear_ratio = 1;

volatile vesc_calc_data calc_data = {
		-1.0,
		-1.0,
		-1.0,
		-1.0,
		-1.0
};


void vesc_calculate_can_data() {
	static timer_ticks_t consumption_ticks = 0;

	calc_data.speed_meter_per_sec = wheel_diam * M_PI * (can_values.rpm / (motor_poles / 2.0) / gear_ratio / 60.0);
	calc_data.speed_kmh = calc_data.speed_meter_per_sec * 3.6;
	calc_data.power_in = can_values.v_in * can_values.current_in;
	calc_data.power_motor = can_values.v_in * can_values.current_motor;
	calc_data.distance_meter = can_values.tachometer_abs * wheel_diam * M_PI / gear_ratio / motor_poles / 3.0;

	if(systicks - consumption_ticks > 1000) {


		consumption_ticks = systicks;
	}


}

