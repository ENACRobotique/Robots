/*
 * Odometry.cpp
 *
 *  Created on: 27 avr. 2017
 *      Author: darian
 */

#include "OdometryController.h"

#include "Arduino.h" //TODO : change to Motor.h
#include "params.h"
extern "C" {
	#include "utils.h"
}


//#define ENTRAXE 7865.56f
#define UPDATE_PERIOD 0.02f

OdometryController Odometry = OdometryController();

OdometryController::OdometryController() {
	_posX = 0;
	_posY = 0;
	_thetaRad = 0;
	_nbIncLeft = 0;
	_nbIncRight = 0;
	_prevL = 0;
	_prevNbIncLeft = 0;
	_prevNbIncRight = 0;
	_leftAcc = 0;
	_rightAcc = 0;
	_speedLeft = _speedRight = 0;

	pinMode(ODO_S_LEFT, INPUT_PULLUP);
	pinMode(ODO_S_RIGHT, INPUT_PULLUP);
	pinMode(ODO_I_LEFT, INPUT_PULLUP);					//maybe useless
	pinMode(ODO_I_RIGHT, INPUT_PULLUP);					//maybe useless
	//attachInterrupt(ODO_I_LEFT, ISRLeft, RISING);
	//attachInterrupt(ODO_I_RIGHT, ISRRight, RISING);

	//_odometryTimer.begin(updatePosition, UPDATE_PERIOD*1000000); //Conversion of period in micro sec
}

void OdometryController::init(double posXi, double posYi, double thetaRadi) {
	_posX = posXi;
	_posY = posYi;
	_thetaRad = thetaRadi;
}

void OdometryController::razIncs() {
	_leftAcc = 0;
	_rightAcc = 0;
}

long OdometryController::getLength(){
	return (_leftAcc + _rightAcc)/2;
}

void OdometryController::updatePosition() {
	/* Update total increment for the trajectory*/
	_leftAcc += _nbIncLeft;
	_rightAcc += _nbIncRight;

	/*Serial.print(_leftAcc);
	Serial.print("\t");
	Serial.println(_rightAcc);*/


	_speedLeft = _nbIncLeft;
	_speedRight = _nbIncRight;

	double speed = (_speedLeft + _speedRight) / 2;

	_thetaRad = constrainAngle(_thetaRad + (_nbIncRight - _nbIncLeft) / RAD_TO_INC);

	_nbIncLeft = 0;
	_nbIncRight = 0;

	double dx = speed * cos(_thetaRad);
	double dy = speed * sin(_thetaRad);
	_posX += dx/MM_TO_INC;
	_posY += dy/MM_TO_INC;

	/*Serial.print("x=");
	Serial.print(_posX);
	Serial.print("\ty=");
	Serial.print(_posY);
	Serial.print("\ttheta=");*/

	static unsigned long time = millis();
	if ((millis() - time) > 50)
		{
			/*Serial.print(time);
			Serial.print("\t");*/
			//Serial.println(_thetaRad);
			time = millis();
		}
}

void OdometryController::ISRLeft() {
	if(digitalRead(ODO_S_LEFT)) {
		_nbIncLeft++;
	} else {
		_nbIncLeft--;
	}
}

void OdometryController::ISRRight() {
	if(digitalRead(ODO_S_RIGHT)) {
		_nbIncRight--;
	} else {
		_nbIncRight++;
	}
}



OdometryController::~OdometryController() {
}



