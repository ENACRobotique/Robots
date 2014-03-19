#ifndef _MOTOR_H
#define _MOTOR_H

typedef struct {
    int speed;
    int setpoint;

    unsigned int lastticksquery;
} motor_t;

void motor_init(motor_t *m, unsigned char pwm, int bank, int pin);
void motor_update(motor_t *m, int pwm_speed);
int motor_getticks(motor_t *m);

#endif
