#include <lpc214x.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <gpio.h>
#include <ime.h>
#include <sys_time.h>
#include "lib_superBus.h"
#include "lib_sbDebug.h"
#include "network_cfg.h"

uint8_t traj_extract_idx = 0;
sTrajElRaw_t traj_blue[] = {
        {7.50, 100.00, 88.75, 112.90, 82.26, 90.00, 105.00, 8.00, 16.43, 0, 0},
        {97.58, 102.44, 82.42, 57.56, 47.37, 90.00, 55.00, 8.00, 38.89, 0, 1},
        {93.66, 62.11, 20.00, 100.00, 82.83, 25.00, 95.00, 0.00, 0.00, 0, 2}
};

int main(void) {
    unsigned int prevLed = 0;
    sMsg msg;

    gpio_init_all();  // use fast GPIOs

    sb_init();

    // sortie LED
    gpio_output(1, 24);
    gpio_write(1, 24, 0); // LED on

    gpio_output(0, 31);
    gpio_write(0, 31, 0); // LED on

    // init time management
    sys_time_init();

    global_IRQ_enable();

    // send position update
    sb_printDbg("start io");

    while(millis() < 500);

    msg.header.destAddr = ADDRI_MAIN_PROP;
    msg.header.type = E_POS;
    msg.header.size = sizeof(sPosPayload);
    msg.payload.pos.x = traj_blue[0].p1_x;
    msg.payload.pos.y = traj_blue[0].p1_y;
    msg.payload.pos.theta = 0.;
    msg.payload.pos.id = 0; // primaire
    sb_send(&msg);

    // main loop
    while(1) {
        sys_time_update();

        sb_routine();

        if( traj_extract_idx < sizeof(traj_blue)/sizeof(*traj_blue) ) {
            msg.header.destAddr = ADDRI_MAIN_PROP;
            msg.header.type = E_TRAJ;
            msg.header.size = sizeof(sTrajElRaw_t);
            memcpy(&msg.payload.traj, &traj_blue[traj_extract_idx], sizeof(sTrajElRaw_t));
            while(sb_send(&msg) < 0);

            traj_extract_idx++;
        }

        if(millis() - prevLed >= 250) {
            prevLed = millis();

            gpio_toggle(1, 24);
            //      gpio_write(0, 31, gpio_read(1, 24));
        }
    }

    return 0;
}
