#include <targets/LPC2000.h>

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
  PWMMCR = 0; // no particular action on matches
  PWMTCR = 0; // counter and timers disabled
  PWMPCR = 0; // all pwm in single edged mode, disabled

  PWMMCR = BIT(1);  // reset timer on match0

  PWMPR = prescale; // prescale of the PCLK (PCLK/(prescale+1))
  PWMMR0 = range; // defines the period of the pwm pattern

  PWMTCR |= BIT(0) | BIT(3); // counter and PWM enable
}

void pwm_enable(unsigned char pwm, unsigned long val) {
  // bind pwm# to pin
  if(pwm==5) {
    PINSEL1 &= ~(3<<pwm_pinshift[pwm-1]);
    PINSEL1 |= 1<<pwm_pinshift[pwm-1];
  }
  else {
    PINSEL0 &= ~(3<<pwm_pinshift[pwm-1]);
    PINSEL0 |= 2<<pwm_pinshift[pwm-1];
  }

  // enable pwm
  PWMPCR |= BIT(8+pwm);

  // set new value
  pwm_update(pwm, val);
}

void pwm_update(unsigned char pwm, unsigned long val) {
  if(pwm<=3)
    *(&PWMMR0+pwm) = val;
  else
    *(&PWMMR4+pwm-4) = val;

  PWMLER |= BIT(pwm);
}
