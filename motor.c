#include <targets/LPC2000.h>

#include "gpio.h"
#include "pwm.h"

#include "motor.h"

void motor_init(motor_t *m, unsigned char pwm, int bank, int pin) {
  m->pwm = pwm;
  m->bank = bank;
  m->pin = pin;
  
  pwm_enable(m->pwm, 0);

  gpio_output(m->bank, m->pin);
  gpio_write(m->bank, m->pin, 0); // forward
}

void motor_update(motor_t *m, int pwm_speed) {
  if(pwm_speed>=0) {
    gpio_write(m->bank, m->pin, 0);
    pwm_update(m->pwm, pwm_speed);
  }
  else {
    gpio_write(m->bank, m->pin, 1);
    pwm_update(m->pwm, -pwm_speed);
  }
}
