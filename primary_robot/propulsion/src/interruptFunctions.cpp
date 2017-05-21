/*
 * interruptFunctions.cpp
 *
 *  Created on: 6 mai 2017
 *      Author: fabien
 */

#include "interruptFunctions.h"
#include "MotorController.h"
#include "OdometryController.h"
#include "TrajectoryManagerClass.h"
#include "InputOutputs.h"
#include "params.h"

void isrLeft() {
	Odometry.ISRLeft();
}

void isrRight() {
	Odometry.ISRRight();
}
/*
void isrTiretteRising() {
	IOs.tiretteRising();
}

void isrTiretteFalling() {
	IOs.tiretteFalling();
}

void isrColorRising() {
	IOs.colorRising();
}

void isrColorFalling() {
	IOs.colorFalling();
}
*/
void setupInterrupts() {
	attachInterrupt(ODO_I_LEFT, isrLeft, RISING);
	attachInterrupt(ODO_I_RIGHT, isrRight, RISING);
	//attachInterrupt(TIRETTE, isrTiretteRising, RISING);
	//attachInterrupt(TIRETTE, isrTiretteFalling, FALLING);
	//attachInterrupt(COLOR, isrColorRising, RISING);
	//attachInterrupt(COLOR, isrColorFalling, FALLING);

}
