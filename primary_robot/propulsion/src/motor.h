/*
 * motor.h
 *
 *  Created on: 6 mars 2017
 *      Author: fabien
 */
#include "Arduino.h"
#include "odometry.h"
#include "params.h"
#ifndef MOTOR_H_
#define MOTOR_H_

#define CLAMP(m, n, M) min(max((m), (n)), (M))

#define MAX_SPEED 10000   //in increments per seconds
#define ACCEL 15000        //in increments per second square

#define KP_DIST 0.3
#define KI_DIST 0.05
#define KD_DIST 0.1

#define KP_ORIENT 0.3
#define KI_ORIENT 0.05
#define KD_ORIENT 0.1

#define MIN_PWM 40

void initMotors();
void computeTrajParameters(long targetLength, long targetTheta);
void controlMotors(long L, long nbIncLeft, long nbIncRight, int speedLeft, int speedRight, double dt);


#endif /* MOTOR_H_ */
