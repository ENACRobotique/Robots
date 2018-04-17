#include "Arduino.h"

#include "src/messages.h"
#include "src/MotorController.h"
#include "src/OdometryController.h"
#include "src/TrajectoryManagerClass.h"
#include "src/InputOutputs.h"
#include "src/interruptFunctions.h"
#include "src/params.h"
#include "src/messageHandlers.h"

unsigned long time = 0;
char ledState = 0;
volatile long acc = 0;
sMessageDown msgDown;
sMessageUp msgUp;
IntervalTimer odometryTimer;
IntervalTimer positionReportTimer;
bool isInit = false;



void updateOdometry() {
	Odometry.updatePosition();
	TrajectoryManager.testRecalage();
	if(Motors.isAtDestination()) {
		TrajectoryManager.computeNextStep();
	}
	Motors.controlMotors();
}

void setup()
{
	Serial.begin(115200);
	Odometry.init(0,0,0);
	Motors.init();
	setupInterrupts();

	odometryTimer.begin(updateOdometry, UPDATE_PERIOD * 1000000);

    message_init(115200);
    IOs.init();

	pinMode(13, OUTPUT);
	Serial.println("start Ness !");


	digitalWrite(13, HIGH);
	delay(2000);
	digitalWrite(13, LOW);
    delay(2000);

/*
    IOs.setLauncherSpeed(100);
    delay(2000);
    IOs.setLauncherSpeed(0);
    IOs.setPickerSpeed(500);
    delay(2000);
    IOs.setPickerSpeed(0);
    IOs.setServoPosition(SERVO_ROCKET, ROCKET_LAUNCH);
    IOs.setServoPosition(SERVO_CANNON_BARRIER, CANNON_BARRIER_OPENED);
*/
    /*int ret;
    Serial.print("toto0");
    TrajectoryManager.addPoint(Point3D(300,0), &ret);
    Serial.print("toto1");*/

}

// The loop function is called in an endless loop
void loop()
{
	if (message_recieve(&msgDown) == 1){
		if (!isInit){
			isInit = true;
			positionReportTimer.begin(reportPosition, REPORT_POSITION_TIMER);
		}
		Serial.print("\nJ'ai un message ! de type ");
		Serial.println(msgDown.type);
		handleMessage(msgDown);
	}
	if(millis() - time > 500) {
		digitalWrite(13, !ledState);
		ledState = !ledState;
		time = millis();
	}
}



