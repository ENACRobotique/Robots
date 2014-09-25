#ifndef _MOTOR_H
#define _MOTOR_H

typedef struct {
#ifdef ARCH_LPC21XX
    // pwm channel
    unsigned char pwm;

    // gpio pin for direction
    int bank;
    int pin;
#elif defined(ARCH_X86_LINUX)
    int speed;
    int setpoint;

    unsigned int lastticksquery;
#endif
} motor_t;

void motor_init(motor_t *m, unsigned char pwm, int bank, int pin);
void motor_update(motor_t *m, int pwm_speed);
#ifdef ARCH_X86_LINUX
int motor_getticks(motor_t *m);
#endif

#endif
