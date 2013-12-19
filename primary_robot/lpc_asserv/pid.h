#ifndef _PID_H
#define _PID_H

typedef struct {
    unsigned char shift;

    unsigned int KP, KI, KD;
    unsigned int I_max;
    int maxErr, sum, maxSum;
    int lastProcessValue;
} pid_t;

// Proportional factor (KP)
// Integral factor (KI=KP*T/Ti)
// Derivative factor (KD=KP*Td/T)
void  pid_init  (pid_t *p, unsigned int KP, unsigned int KI, unsigned int KD, unsigned int I_max, unsigned char shift);
void pid_reset (pid_t *p);
int  pid_update(pid_t *p, int setPoint, int processValue);

#endif
