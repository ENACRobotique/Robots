#ifdef ARCH_LPC21XX
#include <gpio.h>
#include <lpc214x.h>
#include <pins.h>
#include <pwm.h>
#endif

#include "tools.h"

#include "motor.h"

#ifdef ARCH_LPC21XX
void motor_init(motor_t* m, unsigned char pwm_ch, int dir_bank, int dir_pin) {
    m->pwm_ch = pwm_ch;
    m->dir_bank = dir_bank;
    m->dir_pin = dir_pin;

    pwm_enable(m->pwm_ch, 0);
    gpio_output(m->dir_bank, m->dir_pin);
    gpio_write(m->dir_bank, m->dir_pin, 0);
    gpio_enable(m->dir_bank, m->dir_pin);
}
#elif defined(ARCH_X86_LINUX)

void motor_init(motor_t* m) {
    m->speed = 0;
    m->setPoint = 0;
}

#endif

/**
 * Updates motor command
 *   cmd argument is signed
 */
void motor_update(motor_t* m, int cmd) {
#ifdef ARCH_LPC21XX
    if (cmd >= 0) {
        pwm_update(m->pwm_ch, cmd);
        gpio_write(m->dir_bank, m->dir_pin, 1);
    }
    else {
        pwm_update(m->pwm_ch, -cmd);
        gpio_write(m->dir_bank, m->dir_pin, 0);
    }
#elif defined(ARCH_X86_LINUX)
    m->setPoint = (cmd>1024) ? (1024) : (cmd<-1024 ? -1024 : cmd); // Bound cmd between -+1024
#endif
}
