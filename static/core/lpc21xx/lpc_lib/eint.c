#include <lpc214x.h>

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

void eint_init(void (*isr_14)(), void (*isr_17)()) {
  VIC_IntEnClr = BIT(14) | BIT(17);  // disable EINT0 & 3 interrupts

  PCB_PINSEL0 |= 3<<2;  // EINT0 assigned to P0.1
  PCB_PINSEL1 |= 3<<8;  // EINT3 assigned to P0.20

  SCB_EXTMODE |= BIT(0) | BIT(3); // EINT0 & 3 are edge-sensitive
  SCB_EXTPOLAR |= BIT(0) | BIT(3);  // EINT0 & 3 are rising-edge sensitive
  SCB_EXTINT = BIT(0) | BIT(3);  // clear interrupt flag

  VIC_IntSelect &= ~(BIT(14) | BIT(17)); // IRQ (not FIQ)
  // FIXME classify those 2 interrupts as FIQ using VICIntSelect

  VIC_VectCntl2 = BIT(5)|14;
  VIC_VectAddr2 = (unsigned)isr_14;

  VIC_VectCntl3 = BIT(5)|17;
  VIC_VectAddr3 = (unsigned)isr_17;

  SCB_EXTINT = BIT(0) | BIT(3);  // clear interrupt flag
  VIC_IntEnable = BIT(14) | BIT(17);  // enable EINT0 & 3 interrupts
}
