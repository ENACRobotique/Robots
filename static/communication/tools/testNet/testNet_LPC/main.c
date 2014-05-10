#include <lpc214x.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <gpio.h>
#include <ime.h>
#include <sys_time.h>
#include "shared/botNet_core.h"
#include "bn_debug.h"

int main(void) {
    unsigned int prevLed = 0;
    int led_status = 0;
    sMsg msg;

    bn_init();

    gpio_init_all();

    // sortie LED
    gpio_output(1, 24);
    gpio_write(1, 24, 0); // green LED on

    gpio_output(0, 31);
    gpio_write(0, 31, led_status); // orange LED on

    // init time management
    sys_time_init();

    global_IRQ_enable();

    // main loop
    while(1) {
        sys_time_update();

        if(bn_receive(&msg) > 0) {
            gpio_write(0, 31, led_status^=1);
        }

        if(millis() - prevLed >= 250) {
            prevLed += 250;
            gpio_toggle(1, 24); // LED flashing
        }
    }

    return 0;
}
