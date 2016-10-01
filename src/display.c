#include <stdio.h>
#include <math.h>
#include "cmsis_device.h"
#include "timer.h"
#include "ili9341.h"
#include "datatypes.h"
#include "can.h"
#include "vesc.h"

#define BUFFER_LENGTH	5

vesc_calc_data calc_data_last;
mc_values can_values_last = {
	-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1,-1, FAULT_CODE_NONE,
};

uint8_t power_buffer[BUFFER_LENGTH];
uint8_t power_ptr;
uint16_t power;
uint16_t power_last = 1;

// forward declarations
void display_homescreen();


void display_sevenseg(uint8_t num, uint16_t x, uint16_t y, uint32_t foreground) {
	static char buffer[4];
	snprintf(buffer, sizeof(buffer), "%2d", num);
	ILI9341_Puts(x, y, buffer, &Font_7seg, foreground, ILI9341_COLOR_BLACK);
}

void display_sevenseg4(uint16_t num, uint16_t x, uint16_t y, uint32_t foreground) {
	static char buffer[8];
	snprintf(buffer, sizeof(buffer), "%4d", num);
	ILI9341_Puts(x, y, buffer, &Font_7seg, foreground, ILI9341_COLOR_BLACK);
}

void display_float(float f, char *formatStr, uint16_t x, uint16_t y, uint32_t foreground) {
	static char buffer[20];
	int d1 = f;
	float f2 = f - d1;
	int d2 = abs((int)(f2 * 10) % 10);
	snprintf(buffer, sizeof(buffer), formatStr, d1, d2);
	ILI9341_Puts(x, y, buffer, &Font_11x18, foreground, ILI9341_COLOR_BLACK);
}

void show_bar(uint8_t x, uint32_t value) {
    char buffer[20];

    snprintf(buffer, sizeof(buffer), "%6d", value);
    ILI9341_Puts(10, x, buffer, &Font_16x26, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

    uint16_t width = 320.0 / 10000.0 * value;

    ILI9341_DrawFilledRectangle(0, x + 27, width, x + 30, ILI9341_COLOR_WHITE);
    ILI9341_DrawFilledRectangle(width + 1, x + 27, 320, x + 30, ILI9341_COLOR_BLACK);
}

void display_bar(uint16_t y, float max, float current) {
    char buffer[20];

    if(current < 0) current = -current;

    snprintf(buffer, sizeof(buffer), "%3d", (int)(current * 100));
    ILI9341_Puts(0, y, buffer, &Font_16x26, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

    uint16_t width = 320.0 / max * current;

    ILI9341_DrawFilledRectangle(0, y + 27, width, y + 30, ILI9341_COLOR_WHITE);
    ILI9341_DrawFilledRectangle(width, y + 27, 320, y + 30, ILI9341_COLOR_BLACK);
}

void display() {
    static timer_ticks_t display_ticks = 0;

    if (systicks - display_ticks >= 100) {

    	display_homescreen();

        display_ticks = systicks;
    }
}

void display_init() {
    ILI9341_Init();
    ILI9341_Rotate(ILI9341_Orientation_Landscape_2);
}

void display_homescreen() {

	if(fabsf(can_values.v_in - can_values_last.v_in) > 0.1) {
		if(can_values.v_in > (can_mcconf.l_max_vin * 0.85)) {
			display_float(can_values.v_in, "%2d.%1dV", 2, 2, ILI9341_COLOR_GREEN);
		} else if(can_values.v_in > can_mcconf.l_battery_cut_start) {
			display_float(can_values.v_in, "%2d.%1dV", 2, 2, ILI9341_COLOR_YELLOW);
		} else {
			display_float(can_values.v_in, "%2d.%1dV", 2, 2, ILI9341_COLOR_RED);
		}
	}

	if(fabsf(can_values.current_in - can_values_last.current_in) > 0.05) {
		display_float(can_values.current_in, "B %3d.%1dA", 100, 2, ILI9341_COLOR_WHITE);
	}

	if(fabsf(can_values.current_motor - can_values_last.current_motor) > 0.05) {
		display_float(can_values.current_motor, "M %3d.%1dA", 220, 2, ILI9341_COLOR_WHITE);
	}

	if(fabsf(can_values.amp_hours - can_values_last.amp_hours) > 0.0001) {
		display_float(can_values.amp_hours * 1000, "%5d.%1dmAh", 2, 22, ILI9341_COLOR_WHITE);
	}

	if(fabsf(can_values.amp_hours_charged - can_values_last.amp_hours_charged) > 0.0001) {
		display_float(can_values.amp_hours_charged * 1000, "%5d.%1dmAh", 2, 42, ILI9341_COLOR_GREEN2);
	}

	if(calc_data.power_in_filtered != calc_data_last.power_in_filtered) {
		display_sevenseg4(calc_data.power_in_filtered, 320-4*32-17, 190, ILI9341_COLOR_WHITE);
		ILI9341_Puts(320-14, 190+50-18, "W", &Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	}

	if(fabsf(calc_data.speed_kmh - calc_data_last.speed_kmh) > 0.5) {
		display_sevenseg(fabsf(calc_data.speed_kmh), 2, 190, ILI9341_COLOR_WHITE);
		ILI9341_Puts(2+66, 190+50-18, "km/h", &Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	}

	calc_data_last = calc_data;
	can_values_last = can_values;
}


