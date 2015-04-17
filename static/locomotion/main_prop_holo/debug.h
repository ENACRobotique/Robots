#ifndef DEBUG_H
#define DEBUG_H

#ifdef ARCH_LPC21XX

#include <lpc214x.h>
#include <gpio.h>

// Pin for debug
#define PIN_DBG1 26
#define BK_DBG1  1
#define PIN_DBG2 27
#define BK_DBG2  1
#define PIN_DBG3 28
#define BK_DBG3  1
#define PIN_DBG4 29
#define BK_DBG4  1
#define PIN_DBG5 30
#define BK_DBG5  1

//// Pins for small switch
#define PIN_SWTCH1 28
#define BK_SWTCH1  0
#define PIN_SWTCH2 29
#define BK_SWTCH2  0
#define PIN_SWTCH3 30
#define BK_SWTCH3  0

// Definition of five pins of debug
#define DEBUG_1_ON gpio_write(BK_DBG1, PIN_DBG1, 1)
#define DEBUG_1_OFF gpio_write(BK_DBG1, PIN_DBG1, 0)
#define DEBUG_2_ON gpio_write(BK_DBG2, PIN_DBG2, 1)
#define DEBUG_2_OFF gpio_write(BK_DBG2, PIN_DBG2, 0)
#define DEBUG_3_ON gpio_write(BK_DBG3, PIN_DBG3, 1)
#define DEBUG_3_OFF gpio_write(BK_DBG3, PIN_DBG3, 0)
#define DEBUG_4_ON gpio_write(BK_DBG4, PIN_DBG4, 1)
#define DEBUG_4_OFF gpio_write(BK_DBG4, PIN_DBG4, 0)
#define DEBUG_5_ON gpio_write(BK_DBG5, PIN_DBG5, 1)
#define DEBUG_5_OFF gpio_write(BK_DBG5, PIN_DBG5, 0)

#endif

void debug_leds_init(void);
void debug_switches_init(void);

#endif // DEBUG_H
