/*
 * Motor.cpp
 *
 *  Created on: 27 avr. 2017
 *      Author: fabien
 */

#include "Motor.h"
#include "Odometry.h"
#include "params.h"


Motor::Motor() {
	_p = _p1 = _p2 =_p1Real = _p2Real = _pTarget = 0;
	_speed = 0;
	_t1 = _t2 = _tFinal = _t0 = 0;
	_sign = 1;
	_currentMovementType = Straight;
	_currentTime = 0;
	_intErrorLenght = _intErrorTheta = 0;
	_odometry = NULL;
}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

void Motor::init(Odometry* odometry) {

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

void Motor::computeParameters(long target, MovementType movementType) {
	_t0 = millis()/1000.0;
	_odometry->razIncs();
	_pTarget = abs(target);
	if(target > 0) {
		_sign = 1;
	} else {
		_sign = -1;

	}
	_speed = MAX_SPEED;
	_p1 = pow(_speed,2) / (2*ACCEL);
	_p2 = _pTarget - _p1;
	_currentTime = millis();
	_p = 0;
	_currentMovementType = movementType;

	Serial.print("computeParameters  ");
	Serial.println(_pTarget);
}

void Motor::controlMotors() {
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

	Serial.print("Lcons: ");
	Serial.print(lenCons);
	Serial.print("\tlen: ");
	Serial.print(_odometry->getLength());
	Serial.print("\torient: ");
	Serial.println(orientation);
	//Serial.print()

}

long Motor::getConsigne() {
	double t = millis()/1000.0 - _t0;
    if(_p < _p1) {             //accélération
        //serial.printf("accel\n\r");
        _p = (ACCEL * pow(t,2)) / 2;
        _t1 = t;
        _p1Real = _p;
        return _p;
    } else if (_p < _p2) {    //palier de vitesse
        //serial.printf("cruise\n\r");
        _p = _speed*(t-_t1) + _p1Real;
        _p2Real = _p;
        _t2 = t;
        return _p;
    } else {             //décélération
        if(_p >= _pTarget) {
            return _pTarget;
        } else {
            int newX = _p2Real + _speed*(t-_t2) - ((ACCEL * pow((t-_t2),2)) / 2);
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
