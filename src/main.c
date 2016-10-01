#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "timer.h"
#include "can.h"
#include "display.h"
#include "vesc.h"


int main(int argc, char* argv[]) {
    timer_start();

    display_init();
    can_init();

    can_request_vesc(COMM_GET_MCCONF);
    timer_sleep(1000);

    while(1) {
        display();
        can_process();
    }

}




