#include "Arduino.h"
#include "src/odometry.h"
#include "src/messages.h"
#include "src/motorOld.h"

unsigned long time = 0;
char ledState = 0;
volatile long acc = 0;
sMessageDown msgDown;
sMessageUp msgUp;

void setup()
{
	Serial.begin(115200);
    message_init(115200);

	pinMode(13, OUTPUT);
	initMotors();
	setupOdometry();
	digitalWrite(13, HIGH);
	delay(2000);
	digitalWrite(13, LOW);
    computeTrajParameters(20000, 0);
    delay(8000);
    //computeTrajParameters(13500, 27000);
    //delay(6000);
    //computeTrajParameters(-12000, 6000);

}

// The loop function is called in an endless loop
void loop()
{
	if (message_recieve(&msgDown) == 1){
		Serial.print("\nJ'ai un message ! de type ");
		Serial.println(msgDown.type);
	}
	if(millis() - time > 500) {
		digitalWrite(13, !ledState);
		ledState = !ledState;
		time = millis();
	}
}
