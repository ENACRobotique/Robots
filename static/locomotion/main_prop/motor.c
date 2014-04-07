#include <stdio.h>
#ifdef ARCH_X86_LINUX
#include <math.h>
#include "millis.h"
#include "params.h"
#elif defined(ARCH_LPC21XX)
#include <gpio.h>
#include <pwm.h>
#endif

#include "motor.h"

void motor_init(motor_t *m, unsigned char pwm, int bank, int pin) {
#ifdef ARCH_LPC21XX
    m->pwm = pwm;
    m->bank = bank;
    m->pin = pin;

    pwm_enable(m->pwm, 0);

    gpio_output(m->bank, m->pin);
    gpio_write(m->bank, m->pin, 0); // forward
#elif defined(ARCH_X86_LINUX)
    m->speed = 0;
    m->setpoint = 0;

    m->lastticksquery = millis();
#endif
}

void motor_update(motor_t *m, int pwm_speed) {
#ifdef ARCH_LPC21XX
    if(pwm_speed>=0) {
        gpio_write(m->bank, m->pin, 0);
        pwm_update(m->pwm, pwm_speed);
    }
    else {
        gpio_write(m->bank, m->pin, 1);
        pwm_update(m->pwm, -pwm_speed);
    }
#elif defined(ARCH_X86_LINUX)
    m->setpoint = pwm_speed>1024?1024:pwm_speed<-1024?-1024:pwm_speed;
#endif
}

#ifdef ARCH_X86_LINUX
int motor_getticks(motor_t *m){
    unsigned int time = millis();
    int input = m->setpoint; // here, the number of ticks per period is in the same order of magnitude than the pwm control of the motor

#define MOTOR_TIME_CONSTANT (100.) // (ms)

    // very simple motor model... (continuous-time low-pass filter)
    m->speed = (int)(input + (float)(m->speed - input)*exp(-(float)(time - m->lastticksquery)/MOTOR_TIME_CONSTANT));

    m->lastticksquery = time;

    return m->speed;
}
#endif
