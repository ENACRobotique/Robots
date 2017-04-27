/*
 * Motor.h
 *
 *  Created on: 27 avr. 2017
 *      Author: fabien
 */


#ifndef MOTOR_H_
#define MOTOR_H_
#include "Arduino.h"
#include "Odometry.h"

#define MAX_SPEED 10000   //in increments per seconds
#define ACCEL 15000        //in increments per second square

#define KP_DIST 0.3
#define KI_DIST 0.05
#define KD_DIST 0.1

#define KP_ORIENT 0.3
#define KI_ORIENT 0.05
#define KD_ORIENT 0.1

#define MIN_PWM 40

typedef enum{
	Rotation,
	Straight
}MovementType;

class Motor {
public:
	Motor();
	virtual ~Motor();

	void init(Odometry* odometry);
	void computeParameters(long target, MovementType);
	void controlMotors();

protected:
	long getConsigne();
	long getLenghtConsigne();
	long getThetaConsigne();


private:
	double _speed;
	double _t1, _t2, _tFinal, _t0;
	long _p1, _p2, _pTarget, _p1Real, _p2Real, _p;
	bool _sign;
	long _currentTime;
	MovementType _currentMovementType;

	int _intErrorLenght, _intErrorTheta;

	Odometry* _odometry;

};

#endif /* MOTOR_H_ */
