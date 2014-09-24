#ifndef MOTOR_H
#define MOTOR_H

#include "debug.h"
#include "param.h"

#ifdef DVLPT_BOARD
#define RAMPE (1024./7.*1000)
#else
// TODO
#endif



typedef enum eMotorOperation{Drive, FreeWheel, Braking }eMotorOperation;

void controlMotor( int pwmCmd, int dir, eMotorOperation motOp);


#endif // MOTOR_H
