/*
 * Motor.cpp
 *
 *  Created on: 27 avr. 2017
 *      Author: fabien
 */

#include "Motor.h"

Motor::Motor(Odometry* odometry) {
	_p = _p1 = _p2 =_p1Real = _p2Real = _pTarget = 0;
	_speed = 0;
	_t1 = _t2 = _tFinal = 0;
	_sign = true;
	_currentMovementType = Straight;
	_currentTime = 0;
	_intErrorLenght = _intErrorTheta = 0;
	_odometry = odometry
}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

void Motor::initMotors(MotorConfiguration motorConfiguration) {
	_motorConfiguration = motorConfiguration;

	pinMode(motorConfiguration.pinDirLeft, OUTPUT);
	pinMode(motorConfiguration.pinDirRight, OUTPUT);
	pinMode(motorConfiguration.pinPwmLeft, OUTPUT);
	pinMode(motorConfiguration.pinPwmRight, OUTPUT);

	analogWriteFrequency(motorConfiguration.pinPwmLeft, 20000);
	analogWriteFrequency(motorConfiguration.pinPwmRight, 20000);
	analogWriteResolution(10);

	analogWrite(motorConfiguration.pinPwmLeft, 0);
	analogWrite(motorConfiguration.pinPwmRight, 0);
}

void Motor::computeParameters(long target, MovementType movementType) {
	_pTarget = target;
	if(target > 0) {
		_sign = true;
	} else {
		_sign = false;
	}
	_speed = MAX_SPEED;
	_p1 = pow(speed,2) / (2*ACCEL);
	_p2 = _pTarget - _p1;
	_currentTime = millis();
	_p = 0;
	_currentMovementType = movementType;
}

void Motor::controlMotors() {
	int speedRobot = (_odometry->getLeftSpeed() + _odometry->getRightSpeed())/2;
	long lenCons = getLenghtConsigne();
	int lenError = lenCons - _odometry->getLength();
	_intErrorLenght += lenError;
	double lenghtCommand =  lenError * KP_DIST + _intErrorLenght*KI_DIST - KD_DIST * speedRobot;

	int orientation = _odometry->getAccIncLeft() - _odometry->getAccIncRight();
	int orientationSpeed = _odometry->getLeftSpeed() - _odometry->getRightSpeed();
	long thetaCons = getThetaConsigne();
	int thetaError = thetaCons - orientation;
	_intErrorTheta += thetaError;
	double thetaCommand = thetaError * 0.3 + _intErrorTheta*KI_ORIENT - KD_ORIENT * orientationSpeed;

	double leftCommand = lenghtCommand - thetaCommand;
	double rightCommand = lenghtCommand + thetaCommand;

	if(abs(rightCommand) < MIN_PWM) {
		rightCommand = 0;
	}
	if(abs(leftCommand) < MIN_PWM) {
			leftCommand = 0;
		}

	int absRightCommand = min(abs(rightCommand), 1023);
	int absLeftCommand  = min(abs(leftCommand), 1023);

	analogWrite(_motorConfiguration.pinPwmLeft, absLeftCommand);
	analogWrite(_motorConfiguration.pinPwmRight, absRightCommand);
	digitalWrite(_motorConfiguration.pinDirLeft, leftCommand > 0);
	digitalWrite(_motorConfiguration.pinDirRight, rightCommand < 0);
}

long Motor::getConsigne() {
	long t = millis();
    if(_p < _p1) {             //accélération
        //serial.printf("accel\n\r");
        _p = (ACCEL * pow(t,2)) / 2;
        _t1 = t;
        _p1Real = _p;
        return _p;
    } else if (_p < _p2) {    //palier de vitesse
        //serial.printf("cruise\n\r");
        _p = speed*(t-t1) + _p1Real;
        _p2Real = _p;
        _t2 = t;
        return _p;
    } else {             //décélération
        if(_p >= _pTarget) {
            return _pTarget;
        } else {
            int newX = _p2Real + speed*(t-_t2) - ((ACCEL * pow((t-_t2),2)) / 2);
            if(newX < _p) {
                return _pTarget;
            } else {
                _p = newX;
                return _p;
            }
        }
    }
}

long Motor::getLenghtConsigne() {
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

long Motor::getThetaConsigne() {
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
