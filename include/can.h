#ifndef CAN_H_
#define CAN_H_

#include "datatypes.h"

extern mc_configuration can_mcconf;
extern volatile mc_values can_values;


void can_init();
void can_request_vesc(COMM_PACKET_ID);
void can_process();


/* CAN HELP
 *
 * https://www.mikrocontroller.net/topic/182421
 * http://www.diller-technologies.de/stm32.html#can
 * http://www.lp-electronic.com/vesc/control-vesc-via-can-bus-from-external-systems/
 */


#endif
