#include <pins.h>
#include "debug.h"

void debug_leds_init(void) {
    gpio_output(BK_DBG1, PIN_DBG1);
    gpio_output(BK_DBG2, PIN_DBG2);
    gpio_output(BK_DBG3, PIN_DBG3);
    gpio_output(BK_DBG4, PIN_DBG4);
    gpio_output(BK_DBG5, PIN_DBG5);
}

void debug_switches_init(void) {
    gpio_input(BK_SWTCH1, PIN_SWTCH1);
    gpio_input(BK_SWTCH2, PIN_SWTCH2);
    gpio_input(BK_SWTCH3, PIN_SWTCH3);
}
