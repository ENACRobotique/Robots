/*
 * InputOutputs.cpp
 *
 *  Created on: 7 mai 2017
 *      Author: fabien
 */

#include "InputOutputs.h"
#include "Arduino.h"
#include "params.h"
extern "C" {
	#include "utils.h"
}
InputOutputs IOs = InputOutputs();

InputOutputs::InputOutputs() {
	events = 0;
}

InputOutputs::~InputOutputs() {
	// TODO Auto-generated destructor stub
}

void InputOutputs::init() {
	pinMode(BUTEE_LEFT, INPUT_PULLUP);
	pinMode(BUTEE_RIGHT, INPUT_PULLUP);
	pinMode(LED_RED, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE, OUTPUT);
	//pinMode(IHM_BONUS, INPUT);

	pinMode(MOTOR_LAUNCHER, OUTPUT);
	pinMode(MOTOR_PICKER, OUTPUT);
	pinMode(SERVO1, OUTPUT);
	pinMode(SERVO2, OUTPUT);
	pinMode(SERVO3, OUTPUT);
	pinMode(SERVO4, OUTPUT);

	for (int i = 0; i < NB_SERVOS; ++i) {
		_servos[i] = Servo();
	}
	_servos[0].attach(SERVO1);
	_servos[1].attach(SERVO2);
	_servos[2].attach(SERVO3);
	_servos[3].attach(SERVO4);


	setServoPosition(SERVO_CANNON_BARRIER, CANNON_BARRIER_CLOSED);
	IOs.setServoPosition(SERVO_ROCKET, ROCKET_IDLE);
}

void InputOutputs::processActions() {
	if(events && BIT(TiretteEvent)) {
		Serial.print("tirette : ");
		Serial.println(events && BIT(TiretteState));
	}
	if(events && BIT(ColorEvent)) {
		Serial.print("color !");
		Serial.println(events && BIT(ColorState));
	}

	events = 0;
}

void InputOutputs::setLedColor(int r, int g, int b) {
	analogWrite(LED_RED, r);
	analogWrite(LED_GREEN, g);
	analogWrite(LED_BLUE, b);
}

void InputOutputs::setLauncherSpeed(int speed) {
	analogWrite(MOTOR_LAUNCHER, speed);
}

void InputOutputs::setPickerSpeed(int speed) {
	analogWrite(MOTOR_PICKER, speed);
}

void InputOutputs::setServoPosition(int servoNb, int position) {
	int index = -1;
	switch(servoNb) {
		case SERVO1:
			index = 0;
			break;
		case SERVO2:
			index = 1;
			break;
		case SERVO3:
			index = 2;
			break;
		case SERVO4:
			index = 3;
			break;

	}
	_servos[index].write(position);
}

void InputOutputs::tiretteRising() {
	events |= BIT(TiretteEvent);
	events |= BIT(TiretteState);
}

void InputOutputs::tiretteFalling() {
	events |= BIT(TiretteEvent);
	events &= ~BIT(TiretteState);
}

void InputOutputs::colorRising() {
	events |= BIT(ColorEvent);
	events |= BIT(ColorState);
}

void InputOutputs::launchRocket() {
	setServoPosition(SERVO_ROCKET, 150);
}

void InputOutputs::colorFalling() {
	events |= BIT(ColorEvent);
	events &= ~BIT(ColorState);
}

int InputOutputs::isRecaled() {
	if(!digitalRead(BUTEE_LEFT) && !digitalRead(BUTEE_RIGHT)) {
		return true;
	}
	return false;
}
