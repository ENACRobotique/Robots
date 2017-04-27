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


Odometry::Odometry(double posXi, double posYi, double thetaRadi) {
	_posX = posXi;
	_posY = posYi;
	_thetaRad = thetaRadi;
	_nbIncLeft = 0;
	_nbIncRight = 0;
	_prevL = 0;
	_prevNbIncLeft = 0;
	_prevNbIncRight = 0;
	_leftAcc = 0;
	_rightAcc = 0;

	pinMode(ODO_S_LEFT, INPUT_PULLUP);
	pinMode(ODO_S_RIGHT, INPUT_PULLUP);
	pinMode(ODO_I_LEFT, INPUT_PULLUP);					//maybe useless
	pinMode(ODO_I_RIGHT, INPUT_PULLUP);					//maybe useless
	/*attachInterrupt(ODO_I_LEFT, ISRLeft, RISING);
	attachInterrupt(ODO_I_RIGHT, ISRRight, RISING);*/

	//_odometryTimer.begin(updatePosition, UPDATE_PERIOD*1000000); //Conversion of period in micro sec
}

void Odometry::razIncs() {
	_leftAcc = 0;
	_rightAcc = 0;
}

void Odometry::updatePosition() {
	/* Update total increment for the trajectory*/
	_leftAcc += _nbIncLeft;
	_rightAcc += _nbIncRight;

	/*Compute speed*/
	long L = (_leftAcc + _rightAcc) / 2;
	double speed = L - _prevL;
	int speedLeft = _nbIncLeft;
	int speedRight = _nbIncRight;

	/*Reset incr calculated since last update*/
	_nbIncLeft = 0;
	_nbIncRight = 0;

	// TODO : controlMotors(L, _leftAcc, _rightAcc, speedLeft, speedRight, UPDATE_PERIOD);

	/*Store current length*/
	_prevL = L;

	/*Compute new position*/
	_thetaRad = (_rightAcc - _leftAcc) / ENTRAXE;
	double dx = speed * cos(_thetaRad);
	double dy = speed * sin(_thetaRad);
	_posX += dx;
	_posY += dy;
}

void Odometry::leftIncr() {
	_nbIncLeft++;
}

void Odometry::leftDecr() {
	_nbIncLeft--;
}

void Odometry::rightIncr() {
	_nbIncRight++;
}

void Odometry::rightDecr() {
	_nbIncRight--;
}


Odometry::~Odometry() {
}



