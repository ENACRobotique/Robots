#ifndef _MAIN_H
#define _MAIN_H

#include <Servo.h>

#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG

#ifdef DEBUG
extern volatile unsigned int update;
#endif

extern const int ligPinLft;
extern const int ligPinRgt;

extern const int optPinDis;

extern Servo servoDir;
extern const int servoPinDir;
extern const int motorPinDir;
extern const int motorPinPwm;

#define CLAMP(m, n, M) min(max((m), (n)), (M))
#define BIT(b) (1<<(b))

#endif

