/*
 * Odometry.cpp
 *
 *  Created on: 27 avr. 2017
 *      Author: darian
 */

#include "Odometry.h"

#include "Arduino.h" //TODO : change to Motor.h
#include "params.h"

#define ENTRAXE 7865.56f
#define UPDATE_PERIOD 0.02f

Odometry::Odometry() {
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

void Odometry::init(double posXi, double posYi, double thetaRadi) {
	_posX = posXi;
	_posY = posYi;
	_thetaRad = thetaRadi;
}

void Odometry::razIncs() {
	_leftAcc = 0;
	_rightAcc = 0;
}

long Odometry::getLength(){
	return (_leftAcc + _rightAcc)/2;
}

void Odometry::updatePosition() {
	/*Serial.print(_nbIncLeft);
	Serial.print("\t");
	Serial.println(_nbIncRight);*/
	/* Update total increment for the trajectory*/
	_leftAcc += _nbIncLeft;
	_rightAcc += _nbIncRight;

	_speedLeft = _nbIncLeft;
	_speedRight = _nbIncRight;

	/*Compute length*/
	long L = getLength();
	//double speed = L - _prevL;
	double speed = (_speedLeft + _speedRight) / 2;

	/*Reset incr calculated since last update*/



	// TODO : controlMotors(L, _leftAcc, _rightAcc, speedLeft, speedRight, UPDATE_PERIOD);

	/*Store current length*/
	//_prevL = L;

	/*Compute new position*/
	//_thetaRad = (_rightAcc - _leftAcc) / ENTRAXE;
	_thetaRad += (_nbIncRight - _nbIncLeft) / ENTRAXE;

	_nbIncLeft = 0;
	_nbIncRight = 0;

	double dx = speed * cos(_thetaRad);
	double dy = speed * sin(_thetaRad);
	_posX += dx;
	_posY += dy;
}

void Odometry::ISRLeft() {
	if(digitalRead(ODO_S_LEFT)) {
		_nbIncLeft++;
	} else {
		_nbIncLeft--;
	}
}

void Odometry::ISRRight() {
	if(digitalRead(ODO_S_RIGHT)) {
		_nbIncRight--;
	} else {
		_nbIncRight++;
	}
}



Odometry::~Odometry() {
}



