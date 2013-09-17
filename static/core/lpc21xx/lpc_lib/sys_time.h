#ifndef _SYS_TIME_H
#define _SYS_TIME_H

#define PCLK (30000000)
#define T0_PCLK_DIV (1)

#define TIME_TICKS_PER_SEC (PCLK / T0_PCLK_DIV)
#define TIME_TICKS_PER_MSEC (PCLK / (1000 * T0_PCLK_DIV))
#define TIME_TICKS_PER_USEC (PCLK / (1000000 * T0_PCLK_DIV))

void sys_time_init();
void sys_time_update();

unsigned int millis();
unsigned int micros();

/*
// example
unsigned int prev = 0;
if(millis() - prev > 60) {
  prev += 60;
  // do something each 60 milliseconds
}
*/

#endif
