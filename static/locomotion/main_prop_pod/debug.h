#ifndef DEBUG_H
#define DEBUG_H

#include <lpc214x.h>
#include <gpio.h>

// Pin for debug
#define PIN_DBG1 10
#define BK_DBG1  0
#define PIN_DBG2 12
#define BK_DBG2  0
#define PIN_DBG3 14
#define BK_DBG3  0
#define PIN_DBG4 16
#define BK_DBG4  0
#define PIN_DBG5 18
#define BK_DBG5  0

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

void debug_init(void);

#endif // DEBUG_H
