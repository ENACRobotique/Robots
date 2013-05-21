#include <targets/LPC2000.h>

#include "sys_time.h"

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

void t0_isr() {
/*  if(T0IR&BIT(0)) {
    // match0
  }*/

  T0IR = T0IR;  // ack interrupts
  VICVectAddr = 0;  // update priority hardware
}

void sys_time_init() {
  // timer0 disable and reset
  T0TCR = BIT(1);

  T0CTCR = 0;  // timer mode on PCLK rising edge

  T0PR = T0_PCLK_DIV - 1; // prescale factor

  T0CCR = 0;  // disable capture

#if 0
  // configure match0
  T0MR0 = range;
  T0MCR &= ~7;
  T0MCR |= BIT(0) | BIT(1);  // generate an interrupt and reset TC on Match0

  // configure external match
  PINSEL0 &= ~(3<<6);
  PINSEL0 |= 2<<6;  // P0.3 => Match0.0
  T0EMR |= 3<<4;
#else
  T0MCR = 0;  // disable match

  T0EMR = 0;  // disable external match
#endif

  // configure interruption
#if 0
  VICIntEnClr = BIT(4); // disable this irq

  VICIntSelect &= ~BIT(4); // standard irq

  VICVectCntl4 = BIT(5)|4 /* timer0 */; // assign timer0 irq to VIC vector channel #4 and enable
  VICVectAddr4 = (unsigned)t0_isr;

  VICIntEnable = BIT(4); // (re)enable the timer0 irq
#endif

  // timer0 enable
  T0TCR = BIT(0);
}

unsigned int last_time_ticks = 0;
unsigned int cpu_time_sec = 0;

void sys_time_update() {
  unsigned int diff = T0TC - last_time_ticks;  // overflow proof
  if(diff > TIME_TICKS_PER_SEC) {
    cpu_time_sec++;
    last_time_ticks += TIME_TICKS_PER_SEC;
  }
}

unsigned int millis() {
  unsigned int diff = T0TC - last_time_ticks;
  return cpu_time_sec*1000 + diff/TIME_TICKS_PER_MSEC;
}

unsigned int micros() {
  unsigned int diff = T0TC - last_time_ticks;
  return cpu_time_sec*1000000 + diff/TIME_TICKS_PER_USEC;
}
