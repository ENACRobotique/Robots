#ifndef MOTOR_H
#define MOTOR_H

typedef struct {
#ifdef ARCH_LPC21XX
    // pwm channel
    unsigned char pwm_ch;

    // gpio pin for direction
    int dir_bank;
    int dir_pin;

#elif defined(ARCH_X86_LINUX)
    int speed;
    int setPoint;
#endif
} motor_t;

#ifdef ARCH_LPC21XX
void motor_init  (motor_t *m, unsigned char pwm_ch, int bank, int pin);
#elif defined(ARCH_X86_LINUX)
void motor_init  (motor_t *m);
#endif
void motor_update(motor_t *m, int cmd);

#endif // MOTOR_H
