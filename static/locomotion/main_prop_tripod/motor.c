#include <gpio.h>
#include <lpc214x.h>
#include <motor.h>
#include <pins.h>
#include <pwm.h>
#include <tools.h>

void motor_init(motor_t* m, unsigned char pwm_ch, int dir_bank, int dir_pin) {
#ifdef ARCH_LPC21XX
    m->pwm_ch = pwm_ch;
    m->dir_bank = dir_bank;
    m->dir_pin = dir_pin;

    pwm_enable(m->pwm_ch, 0);
    gpio_output(m->dir_bank, m->dir_pin);
    gpio_write(m->dir_bank, m->dir_pin, 0);
    gpio_enable(m->dir_bank, m->dir_pin);
#elif ARCH_X86_LINUX
    m->speed = 0;
    m->setpoint = 0;
#endif
}

/**
 * Updates motor command
 *   cmd argument is signed
 */
void motor_update(motor_t* m, int cmd) {
#ifdef ARCH_LPC21XX
    if (cmd >= 0) {
        pwm_update(m->pwm_ch, cmd);
        gpio_write(m->dir_bank, m->dir_pin, 0);
    }
    else {
        pwm_update(m->pwm_ch, -cmd);
        gpio_write(m->dir_bank, m->dir_pin, 1);
    }
#elif ARCH_X86_LINUX
    m->setPoint = (cmd>1024) ? (1024) : (cmd<-1024 ? -1024 : cmd); // Bound cmd between -+1024
#endif
}
