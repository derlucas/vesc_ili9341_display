#ifndef VESC_H_
#define VESC_H_

#include "datatypes.h"


extern volatile vesc_calc_data calc_data;

void vesc_calculate_can_data();


#endif
