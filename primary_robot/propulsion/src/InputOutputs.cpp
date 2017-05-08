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
	pinMode(TIRETTE, INPUT_PULLUP);
	pinMode(COLOR, INPUT_PULLUP);
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
	_servos[0].attach(SERVO2);
	_servos[0].attach(SERVO3);
	_servos[0].attach(SERVO4);
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
	_servos[servoNb].write(position);
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

void InputOutputs::colorFalling() {
	events |= BIT(ColorEvent);
	events &= ~BIT(ColorState);
}
