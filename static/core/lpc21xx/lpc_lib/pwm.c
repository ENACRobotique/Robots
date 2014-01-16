#include <lpc214x.h>

#include "pwm.h"

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

unsigned char pwm_pinshift[] = {
  0*2, // PWM1 @ P0.0
  7*2, // PWM2 @ P0.7
  1*2, // PWM3 @ P0.1
  8*2, // PWM4 @ P0.8
  10, // PWM5 @ P0.21
  9*2 // PWM6 @ P0.9
};

void pwm_init(unsigned long prescale, unsigned long range) {
  // reset config
  PWM_MCR = 0; // no particular action on matches
  PWM_TCR = 0; // counter and timers disabled
  PWM_PCR = 0; // all pwm in single edged mode, disabled

  PWM_MCR = BIT(1);  // reset timer on match0

  PWM_PR = prescale; // prescale of the PCLK (PCLK/(prescale+1))
  PWM_MR0 = range; // defines the period of the pwm pattern
  PWM_LER = BIT(0); // updates PWM_MR0

  PWM_TCR = BIT(0) | BIT(3); // counter and PWM enable
}

void pwm_enable(unsigned char pwm /* 1-6 */, unsigned long val) {
  // bind pwm# to pin
  if(pwm==5) {
    PCB_PINSEL1 &= ~(3<<pwm_pinshift[pwm-1]);
    PCB_PINSEL1 |= 1<<pwm_pinshift[pwm-1];
  }
  else {
    PCB_PINSEL0 &= ~(3<<pwm_pinshift[pwm-1]);
    PCB_PINSEL0 |= 2<<pwm_pinshift[pwm-1];
  }

  // enable pwm
  PWM_PCR = (PWM_PCR & ~0x8183) | BIT(8+pwm);

  // set new value
  pwm_update(pwm, val);
}

void pwm_update(unsigned char pwm /* 1-6 */, unsigned long val) {
  if(pwm<=3)
    *(&PWM_MR0+pwm) = val;
  else
    *(&PWM_MR4+pwm-4) = val;

  PWM_LER |= BIT(pwm); // marks this pwm for update at the next MR0 match
}
