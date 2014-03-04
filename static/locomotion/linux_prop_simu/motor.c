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
    int input = m->setpoint*4;

#define MOTOR_TIME_CONSTANT (800.) // (ms)

    // very simple motor model... (low-pass filter)
    m->speed = (int)(input + (float)(m->speed - input)*exp(-(float)(time - m->lastticksquery)/MOTOR_TIME_CONSTANT));

    m->lastticksquery = time;

//    printf("gt %p|%i\n", m, m->speed);

    return m->speed;
}
