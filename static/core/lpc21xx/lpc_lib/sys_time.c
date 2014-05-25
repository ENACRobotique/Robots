#include <lpc214x.h>

#include "sys_time.h"

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

#if 0
void t0_isr() { // XXX missing irq nature
/*  if(T0_IR&BIT(0)) {
    // match0
  }*/

  T0_IR = T0_IR;  // ack interrupts
  VIC_VectAddr = 0;  // update priority hardware
}
#endif

void sys_time_init() {
  // timer0 disable and reset
  T0_TCR = BIT(1);

  T0_CTCR = 0;  // timer mode on PCLK rising edge

  T0_PR = T0_PCLK_DIV - 1; // prescale factor

  T0_CCR = 0;  // disable capture

#if 0
  // configure match0
  T0_MR0 = range;
  T0_MCR &= ~7;
  T0_MCR |= BIT(0) | BIT(1);  // generate an interrupt and reset TC on Match0

  // configure external match
  PCB_PINSEL0 &= ~(3<<6);
  PCB_PINSEL0 |= 2<<6;  // P0.3 => Match0.0
  T0_EMR |= 3<<4;
#else
  T0_MCR = 0;  // disable match

  T0_EMR = 0;  // disable external match
#endif

  // configure interruption
#if 0
  VIC_IntEnClr = BIT(4); // disable this irq

  VIC_IntSelect &= ~BIT(4); // standard irq

  VIC_VectCntl4 = BIT(5)|4 /* timer0 */; // assign timer0 irq to VIC vector channel #4 and enable
  VIC_VectAddr4 = (unsigned)t0_isr;

  VIC_IntEnable = BIT(4); // (re)enable the timer0 irq
#endif

  // timer0 enable
  T0_TCR = BIT(0);
}

unsigned int last_time_ticks = 0;
unsigned int cpu_time_sec = 0;

void sys_time_update() {
  unsigned int diff = T0_TC - last_time_ticks;  // overflow proof
  if(diff > TIME_TICKS_PER_SEC) {
    cpu_time_sec++;
    last_time_ticks += TIME_TICKS_PER_SEC;
  }
}

unsigned int millis() {
  unsigned int diff = T0_TC - last_time_ticks;
  return cpu_time_sec*1000 + diff/TIME_TICKS_PER_MSEC;
}

unsigned int micros() {
  unsigned int diff = T0_TC - last_time_ticks;
  return cpu_time_sec*1000000 + diff/TIME_TICKS_PER_USEC;
}
