#include "Arduino.h"

#include "src/messages.h"
#include "src/MotorController.h"
#include "src/OdometryController.h"
#include "src/TrajectoryManagerClass.h"
#include "src/params.h"

unsigned long time = 0;
char ledState = 0;
volatile long acc = 0;
sMessageDown msgDown;
sMessageUp msgUp;
IntervalTimer odometryTimer;

void isrLeft() {
	Odometry.ISRLeft();
}

void isrRight() {
	Odometry.ISRRight();
}
void updateOdometry() {
	Odometry.updatePosition();
	if(Motors.isAtDestination()) {
		TrajectoryManager.computeNextStep();
	}
	Motors.controlMotors();
}

void setup()
{
	Serial.begin(115200);
	Odometry.init(0,0,0);
	Motors.init(&Odometry);
	attachInterrupt(ODO_I_LEFT, isrLeft, RISING);
	attachInterrupt(ODO_I_RIGHT, isrRight, RISING);

	odometryTimer.begin(updateOdometry, UPDATE_PERIOD * 1000000);

    message_init(115200);

	pinMode(13, OUTPUT);
	Serial.println("start !");

	digitalWrite(13, HIGH);
	delay(2000);
	digitalWrite(13, LOW);
	//Motors.computeParameters(20000, Rotation);
    delay(2000);

    //Point3D pt = Point3D(1000,300);
    int ret;
    TrajectoryManager.addPoint(Point3D(100, 0, -1.57), &ret);
    //TrajectoryManager.addPoint(Point3D(150,0,1.57), &ret);
    //TrajectoryManager.addPoint(Point3D(150,0,-1.57), &ret);
    //TrajectoryManager.addPoint(Point3D(300, 300, 0), &ret);
    //TrajectoryManager.addPoint(Point3D(1000, 0), &ret);
    //TrajectoryManager.addPoint(Point3D(2000, 0), &ret);
    //TrajectoryManager.readPoint(&pt, &ret);
    TrajectoryManager.computeNextStep();
}

// The loop function is called in an endless loop
void loop()
{
	if (message_recieve(&msgDown) == 1){
		Serial.print("\nJ'ai un message ! de type ");
		Serial.println(msgDown.type);
	}
	/*Motors.computeParameters(20000, Straight);
	delay(3000);
	Motors.computeParameters(-20000, Straight);
	delay(3000);*/
	//Motors.computeParameters(2200, Straight);
	//delay(10000);
	//Motors.computeParameters(-2200, Straight);
	//delay(10000);
	//Motors.computeParameters(6.283, Rotation);
	//delay(20000);

	if(millis() - time > 500) {
		digitalWrite(13, !ledState);
		ledState = !ledState;
		time = millis();
	}
}



