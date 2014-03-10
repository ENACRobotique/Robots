#include <stdio.h>
#include <math.h>

#include "millis.h"
#include "params.h"

#include "motor.h"

void motor_init(motor_t *m, unsigned char pwm, int bank, int pin) {
    m->speed = 0;
    m->setpoint = 0;

    m->lastticksquery = millis();
}

void motor_update(motor_t *m, int pwm_speed) {
    m->setpoint = pwm_speed;
}

int motor_getticks(motor_t *m){
    int time = millis();
    int input = m->setpoint; // here, the number of ticks per period is in the same order of magnitude than the pwm control of the motor

#define MOTOR_TIME_CONSTANT (100.) // (ms)

    // very simple motor model... (continuous-time low-pass filter)
    m->speed = (int)(input + (float)(m->speed - input)*exp(-(float)(time - m->lastticksquery)/MOTOR_TIME_CONSTANT));

    m->lastticksquery = time;

    return m->speed;
}
