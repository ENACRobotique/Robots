#include <gpio.h>
#include <lpc214x.h>
#include <motor.h>
#include <param.h>
#include <pwm.h>
#include <tools.h>

void motor_init(motor_t* m, unsigned char pwm, int dir_bank, int dir_pin) {
    m->pwm = pwm;
    m->dir_bank = dir_bank;
    m->dir_pin = dir_pin;

    pwm_enable(m->pwm, 0);
    gpio_output(m->dir_bank, m->dir_pin);
    gpio_write(m->dir_bank, m->dir_pin, 0);
    gpio_enable(m->dir_bank, m->dir_pin);
}

void motor_update(motor_t* m, int pwm) {
    if (pwm >= 0) {
        pwm_update(m->pwm, pwm);
        gpio_write(m->dir_bank, m->dir_pin, 0);
    }
    else {
        pwm_update(m->pwm, -pwm);
        gpio_write(m->dir_bank, m->dir_pin, 1);
    }
}
