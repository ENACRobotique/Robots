/*
 * InputOutputs.h
 *
 *  Created on: 7 mai 2017
 *      Author: fabien
 */

#ifndef INPUTOUTPUTS_H_
#define INPUTOUTPUTS_H_
#include "Servo.h"

#define NB_SERVOS 4

enum {
	TiretteEvent,
	TiretteState,
	ColorEvent,
	ColorState
};

class InputOutputs {
public:
	InputOutputs();
	virtual ~InputOutputs();

	void init();
	void processActions();

	void setLedColor(int r, int g, int b);
	void setLauncherSpeed(int speed);
	void setPickerSpeed(int speed);

	void setServoPosition(int servo, int position);
	void setServoPositionMs(int servo, int ms);

	void launchRocket();

	void tiretteRising();
	void tiretteFalling();
	void colorRising();
	void colorFalling();
	int isRecaled();

private:

	Servo _servos[NB_SERVOS];
	int events;

};

extern InputOutputs IOs;

#endif /* INPUTOUTPUTS_H_ */
/*
TIRETTE
COLOR
LED_RED
LED_GREEN
LED_BLUE
IHM_BONUS
MOTOR_LAUNCHER
MOTOR_PICKUP
SERVO1
SERVO2
SERVO3
SERVO4



*/
