/*
 * Motor.cpp
 *
 *  Created on: 27 avr. 2017
 *      Author: fabien
 */

#include "MotorController.h"
#include "OdometryController.h"
#include "params.h"

MotorController Motors = MotorController();

MotorController::MotorController() {
	_p = _p1 = _p2 =_p1Real = _p2Real = _pTarget = _pMax =0;
	_speed = 0;
	_t1 = _t2 = _tFinal = _t0 = 0;
	_sign = 1;
	_currentMovementType = Straight;
	_currentTime = 0;
	_intErrorLenght = _intErrorTheta = 0;
	_odometry = NULL;
	_movementPhase = Stopped;
}

MotorController::~MotorController() {
	// TODO Auto-generated destructor stub
}

void MotorController::init(OdometryController* odometry) {

	pinMode(DIR_LEFT, OUTPUT);
	pinMode(DIR_RIGHT, OUTPUT);
	pinMode(PWM_LEFT, OUTPUT);
	pinMode(PWM_RIGHT, OUTPUT);

	analogWriteFrequency(PWM_LEFT, 20000);
	analogWriteFrequency(PWM_RIGHT, 20000);
	analogWriteResolution(10);

	analogWrite(PWM_LEFT, 0);
	analogWrite(PWM_RIGHT, 0);
	_odometry = odometry;
}

void MotorController::computeParameters(long target, MovementType movementType, double speed) {
	_t0 = millis()/1000.0;
	_odometry->razIncs();
	_pTarget = abs(target);
	if(target > 0) {
		_sign = 1;
	} else {
		_sign = -1;
	}
	speed = min(speed, MAX_SPEED);
	double maxSpeed = sqrt(ACCEL * _pTarget);
	_speed = min(speed, maxSpeed);
	_p1 = pow(_speed,2) / (2*ACCEL);
	_p2 = _pTarget - _p1;
	_p1Real = _p1;
	_p2Real = _p2;
	_currentTime = millis();
	_p = 0;
	_currentMovementType = movementType;

	_movementPhase = Acceleration;

	Serial.print("computeParameters  ");
	Serial.println(_pTarget);
}

void MotorController::controlMotors() {
	int speedRobot = (_odometry->getSpeedLeft() + _odometry->getSpeedRight())/2;
	long lenCons = getLenghtConsigne();
	int lenError = lenCons - _odometry->getLength();
	_intErrorLenght += lenError;
	double lenghtCommand =  lenError * KP_DIST + _intErrorLenght*KI_DIST - KD_DIST * speedRobot;

	int orientation = _odometry->getLeftAcc() - _odometry->getRightAcc();
	int orientationSpeed = _odometry->getSpeedLeft() - _odometry->getSpeedRight();
	long thetaCons = getThetaConsigne();
	int thetaError = thetaCons - orientation;
	_intErrorTheta += thetaError;
	double thetaCommand = thetaError * KP_ORIENT + _intErrorTheta*KI_ORIENT - KD_ORIENT * orientationSpeed;

	double leftCommand = lenghtCommand + thetaCommand;
	double rightCommand = lenghtCommand - thetaCommand;

	if(abs(rightCommand) < MIN_PWM) {
		rightCommand = 0;
	}
	if(abs(leftCommand) < MIN_PWM) {
			leftCommand = 0;
	}

	int absRightCommand = min(abs(rightCommand), 1023);
	int absLeftCommand  = min(abs(leftCommand), 1023);
	//int absRightCommand = min(abs(rightCommand), 250);
	//int absLeftCommand  = min(abs(leftCommand), 250);

	analogWrite(PWM_LEFT, absLeftCommand);
	analogWrite(PWM_RIGHT, absRightCommand);
	digitalWrite(DIR_LEFT, leftCommand > 0);
	digitalWrite(DIR_RIGHT, rightCommand < 0);

	//Serial.print("Lcons: ");
	Serial.println(lenCons);
	//Serial.print(";");
	//Serial.print("\tlen: ");
	//Serial.println(_odometry->getLength());
	//Serial.print("\torient: ");
	//Serial.println(orientation);

}

long MotorController::getConsigne() {
	double t = millis()/1000.0 - _t0;



	switch(_movementPhase) {
		case Stopped:
			return 0;
			break;
		case Acceleration:
			if(_p < _p1) {
				getAccelConsigne(t);
			} else if(_p < _p2){
				Serial.println("Cruise");
				_movementPhase = Cruise;
				getCruiseConsigne(t);
			} else {
				Serial.println("Deceleration");
				_movementPhase = Deceleration;
				getDecelConsigne(t);
			}
			break;
		case Cruise:
			if(_p < _p2) {
				getCruiseConsigne(t);
			} else {
				Serial.println("Deceleration");
				_movementPhase = Deceleration;
				getDecelConsigne(t);
			}
			break;
		case Deceleration:
			getDecelConsigne(t);
			break;
	}

	return _p;
}

long MotorController::getLenghtConsigne() {
	switch (_currentMovementType) {
		case Straight:
			return _sign * getConsigne();
			break;
		case Rotation:
			return 0;
			break;
	}
	return 0;
}

long MotorController::getThetaConsigne() {
	switch (_currentMovementType) {
			case Straight:
				return 0;
				break;
			case Rotation:
				return _sign * getConsigne();
				break;
		}
	return 0;
}

long MotorController::getAccelConsigne(double t) {
	_p = (ACCEL * pow(t,2)) / 2;
	_t1 = t;
	_t2 = t;
	_p1Real = _p;
	_p2Real = _p;
	_pMax = _p;
	return _p;
}

long MotorController::getCruiseConsigne(double t) {
	_p = _speed*(t-_t1) + _p1Real;
	_p2Real = _p;
	_pMax = _p;
	_t2 = t;
	return _p;
}

long MotorController::getDecelConsigne(double t) {

	int newX = _p2Real + _speed*(t-_t2) - ((ACCEL * pow((t-_t2),2)) / 2);
	if(newX < _pMax) {
		return _pTarget;
	} else {
		_p = newX;
		_pMax = _p;
		return _p;
	}
}
