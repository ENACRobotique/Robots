#include <pins.h>
#include "debug.h"

void debug_leds_init(void) {
#ifdef ARCH_LPC21XX
    // set pins for LPC LED in output
    gpio_output(0, 31);
    gpio_output(1, 24);
#endif
}

void debug_switches_init(void) {
#ifdef ARCH_LPC21XX
    //Set pins in input to use switches
    gpio_input(BK_SWTCH1, PIN_SWTCH1);
    gpio_input(BK_SWTCH2, PIN_SWTCH2);
    gpio_input(BK_SWTCH3, PIN_SWTCH3);
#endif
}
