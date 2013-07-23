#ifndef _PWM_H
#define _PWM_H

void pwm_init(unsigned long prescale, unsigned long range);
void pwm_enable(unsigned char pwm, unsigned long val);
void pwm_update(unsigned char pwm, unsigned long val);

#endif
