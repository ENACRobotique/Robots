/*
 * Motor.h
 *
 *  Created on: 27 avr. 2017
 *      Author: fabien
 */


#ifndef MOTOR_H_
#define MOTOR_H_
#include "Arduino.h"
#include "OdometryController.h"

#define MAX_SPEED 10000.0 	  //in increments per seconds
#define ACCEL 15000.0 	    //in increments per second square

#define KP_DIST 0.3
#define KI_DIST 0.05
#define KD_DIST 0.1

#define KP_ORIENT 0.5
#define KI_ORIENT 0//0.05
#define KD_ORIENT 0//0.1

enum{
	KP, KI, KD
};

#define MIN_PWM 40

typedef enum{
	Rotation,
	Straight
}MovementType;

typedef enum{
	Acceleration,
	Cruise,
	Deceleration,
	Stopped
}MovementPhase;

class MotorController {
public:
	MotorController();
	virtual ~MotorController();

	void init();
	void computeParameters(double target, MovementType, double speed = MAX_SPEED);
	void controlMotors();
	bool isAtDestination();
	void setOrientCoeffs();
	void clearOrientCoeffs();

protected:
	long getConsigne();
	long getLenghtConsigne();
	long getThetaConsigne();

	long getAccelConsigne(double t);
	long getCruiseConsigne(double t);
	long getDecelConsigne(double t);
	void setAccel();


private:
	double _speed;
	double _t1, _t2, _tFinal, _t0;
	long _p1, _p2, _pTarget, _p1Real, _p2Real, _p, _pMax;
	long _accel;
	int _sign;
	long _currentTime;
	MovementType _currentMovementType;
	MovementPhase _movementPhase;

	int _intErrorLenght, _intErrorTheta;

	double kOrient[3];

};
extern MotorController Motors;
#endif /* MOTOR_H_ */
