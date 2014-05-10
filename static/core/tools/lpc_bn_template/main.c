#include <stdio.h>
#include <shared/botNet_core.h>
#include <bn_debug.h>
#include <i2c.h>
#include <gpio.h>
#include <ime.h>
#include <sys_time.h>
#include <errno.h>
#include <string.h>
#include <uart0.h>

#include "node_cfg.h"

int main(void) {
    unsigned int time;
    int led0_status = 0, led1_status = 0;
    unsigned int led0_prevT = 0;
    sMsg inMsg;

    gpio_init_all();  // use fast GPIOs

    // status LEDs
    gpio_output(1, 24);
    gpio_write(1, 24, led0_status); // green LED is on when its output is 0

    gpio_output(0, 31);
    gpio_write(0, 31, led1_status); // orange LED is on when its output is 0

    // init time management
    sys_time_init();

    // allow interrupt requests
    global_IRQ_enable();

    // superbus init
    bn_init();
    unsigned int bd = uart0_init(9600);

    bn_printDbg("Hello world, I'm lpc_bn_template!");
    printf("UART0 initialized with baudrate %u\n", bd);

    led0_prevT = millis();

    // main loop
    while(1) {
        sys_time_update();
        time = millis();

        bn_receive(&inMsg);

        if((time - led0_prevT) > 200){
            led0_prevT = time;

            gpio_write(1, 24, led0_status^=1);

            bn_printfDbg("led0_status=%i\n", led0_status);
            printf("led0_status=%i\n", led0_status);
        }
    }

    return 0;
}
