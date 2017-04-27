#include "Arduino.h"

#include "src/messages.h"
#include "src/Motor.h"
#include "src/Odometry.h"
#include "src/params.h"

unsigned long time = 0;
char ledState = 0;
volatile long acc = 0;
sMessageDown msgDown;
sMessageUp msgUp;
Odometry odometry = Odometry();
Motor motor = Motor();
IntervalTimer odometryTimer;

void isrLeft() {
	odometry.ISRLeft();
}

void isrRight() {
	odometry.ISRRight();
}
void updateOdometry() {
	odometry.updatePosition();
	motor.controlMotors();
}

void setup()
{
	Serial.begin(115200);
	odometry.init(0,0,0);
	motor.init(&odometry);
	attachInterrupt(ODO_I_LEFT, isrLeft, RISING);
	attachInterrupt(ODO_I_RIGHT, isrRight, RISING);

	odometryTimer.begin(updateOdometry, UPDATE_PERIOD * 1000000);

    //message_init(115200);

	pinMode(13, OUTPUT);
	Serial.println("start !");

	digitalWrite(13, HIGH);
	delay(2000);
	digitalWrite(13, LOW);
    motor.computeParameters(20000, Rotation);
    delay(3000);

}

// The loop function is called in an endless loop
void loop()
{
	/*if (message_recieve(&msgDown) == 1){
		Serial.print("\nJ'ai un message ! de type ");
		Serial.println(msgDown.type);
	}*/
	motor.computeParameters(20000, Straight);
	delay(3000);
	motor.computeParameters(-20000, Straight);
	delay(3000);
	if(millis() - time > 500) {
		digitalWrite(13, !ledState);
		ledState = !ledState;
		time = millis();
	}
}



