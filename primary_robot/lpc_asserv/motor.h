#ifndef _MOTOR_H
#define _MOTOR_H

typedef struct {
    // pwm channel
    unsigned char pwm;

    // gpio pin for direction
    int bank;
    int pin;
} motor_t;

void motor_init(motor_t *m, unsigned char pwm, int bank, int pin);
void motor_update(motor_t *m, int pwm_speed);

#endif
