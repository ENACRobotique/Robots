#include <targets/LPC2000.h>

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

void eint_init(void (*isr_14)(), void (*isr_17)()) {
  VICIntEnClr = BIT(14) | BIT(17);  // disable EINT0 & 3 interrupts

  PINSEL0 |= 3<<2;  // EINT0 assigned to P0.1
  PINSEL1 |= 3<<8;  // EINT3 assigned to P0.20

  EXTMODE |= BIT(0) | BIT(3); // EINT0 & 3 are edge-sensitive
  EXTPOLAR |= BIT(0) | BIT(3);  // EINT0 & 3 are rising-edge sensitive
  EXTINT |= BIT(0) | BIT(3);  // clear interrupt flag

  VICIntSelect &= ~(BIT(14) | BIT(17)); // IRQ (not FIQ)
  // FIXME classify those 2 interrupts as FIQ using VICIntSelect

  VICVectCntl2 = BIT(5)|14;
  VICVectAddr2 = (unsigned)isr_14;

  VICVectCntl3 = BIT(5)|17;
  VICVectAddr3 = (unsigned)isr_17;

  EXTINT |= BIT(0) | BIT(3);  // clear interrupt flag
  VICIntEnable = BIT(14) | BIT(17);  // enable EINT0 & 3 interrupts
}
