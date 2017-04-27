/*
 * odometry.cpp
 *
 *  Created on: 6 mars 2017
 *      Author: fabien
 */
#include "odometry.h"

#include "motorOld.h"
#include "params.h"
void ISRLeft();
void ISRRight();

#define ENTRAXE 7865.56f
#define UPDATE_PERIOD 0.02f

volatile int nbIncLeft;
volatile int nbIncRight;

long prevNbIncLeft;
long prevNbIncRight;
long prevL;
double posX;
double posY;
double thetaRad;

IntervalTimer odometryTimer;

void updatePosition() {
	static long leftAcc=0;
	static long rightAcc=0;
	leftAcc += nbIncLeft;
	rightAcc += nbIncRight;
	//Serial.printf("l=%d  r=%d\n\r", nbIncLeft, nbIncRight);


    long L = (leftAcc + rightAcc) / 2;
    double speed = L - prevL;

    int speedLeft = nbIncLeft;
    int speedRight = nbIncRight;
    nbIncLeft = 0;
    nbIncRight = 0;

    controlMotors(L, leftAcc, rightAcc, speedLeft, speedRight, UPDATE_PERIOD);

    prevL = L;
    thetaRad = (rightAcc - leftAcc) / ENTRAXE;
    double dx = speed * cos(thetaRad);
    double dy = speed * sin(thetaRad);
    posX += dx;
    posY += dy;
}


void setupOdometry() {
    nbIncLeft = 0;
    nbIncRight = 0;
    prevNbIncLeft = prevNbIncRight = 0;
    posX = posY = thetaRad = prevL = 0;

	pinMode(ODO_S_LEFT, INPUT_PULLUP);
	pinMode(ODO_S_RIGHT, INPUT_PULLUP);
	pinMode(ODO_I_LEFT, INPUT_PULLUP);					//maybe useless
	pinMode(ODO_I_RIGHT, INPUT_PULLUP);					//maybe useless
	attachInterrupt(ODO_I_LEFT, ISRLeft, RISING);
	attachInterrupt(ODO_I_RIGHT, ISRRight, RISING);

	odometryTimer.begin(updatePosition, UPDATE_PERIOD*1000000);
}

void ISRLeft() {
	int state = digitalRead(ODO_S_LEFT);
	//Serial.println(state);
	if(state) {
		nbIncLeft++;
	} else {
		nbIncLeft--;
	}
}

void ISRRight() {
	int state = digitalRead(ODO_S_RIGHT);
	if(state) {
		nbIncRight--;
	} else {
		nbIncRight++;
	}
}

double getX() {
    return posX/29.24;
}

double getY() {
    return posY/29.24;
}

double getTheta() {
    return thetaRad;
}

long getL() {
    return prevL;
}

long getIncLeft() {
    return nbIncLeft;
}

long getIncRight() {
    return nbIncRight;
}
