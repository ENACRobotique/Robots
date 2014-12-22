#ifndef MOTOR_H
#define MOTOR_H

#include "debug.h"
#include "param.h"

#ifdef DVLPT_BOARD
#define RAMPE (1024./7.*1000.)
#else
// TODO
#error TODO unimplemented yet
#endif



typedef enum eMotorOperation{Drive, FreeWheel, Braking }eMotorOperation;
typedef enum eMotorDir{Trigo, Notrigo }eMotorDir;
typedef enum eStateBstr{ChgBstr, DisChgBstr}eStateBstr;

void controlMotor( int pwmCmd, eMotorDir dir, eMotorOperation motOp);


#endif // MOTOR_H
