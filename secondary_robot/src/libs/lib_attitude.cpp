/*
this library contains the different functions useful for the motor and its control
*/

#include "lib_attitude.h"
#include "MPU_6050.h"
#include "../params.h"

//defines
#define ATTITUDE_ASSER_PERIOD 20 // milliseconds

#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

//globals
int _attitudeCon;
int _pinServo;
Servo servoAttitude;

//initializes the PINs for motor speed and direction
//REQUIRES : odoinitHard()
void servoInitHard(int pinservo){
	_pinServo=pinservo;
	servoAttitude.attach(_pinServo);
}

#define KP  1// >>2
#define KI  1// >>8

void attitudeAsser(){

//int angle_x = readInertial();
//_attitudeCon = servoAttitude.read() + angle_x/5;
//int cketuveu = max(10,min(175,(_attitudeCon)));
//servoAttitude.write(cketuveu);

    unsigned long int time=millis();
    static int intEps=0;
    static unsigned long time_prev_asser=millis();
    static int _attitudeCmd=0;
	int eps;


	// asservissement attitude
	if((time-time_prev_asser)>=ATTITUDE_ASSER_PERIOD) {
		if ( (time-time_prev_asser) < ATTITUDE_ASSER_PERIOD+ATTITUDE_ASSER_PERIOD/2 ){
			time_prev_asser = time_prev_asser + ATTITUDE_ASSER_PERIOD;
			//compute error (epsilon)
			int read=readInertial();
			eps = _attitudeCon - read;

			//compute error integral
			intEps= CLAMP( -(64<<4) ,intEps+eps, (64<<4));
			//compute command
//			if(_attitudeCon==0){
//				_attitudeCmd=0;
//			}
//			else{
				_attitudeCmd= servoAttitude.read() - ((KP*eps)>>2) - ((KI*intEps)>>8);
//			}

#ifdef DEBUG_ATTITUDE
Serial.print(0.01);
Serial.print("\t");
Serial.print(_attitudeCon);
Serial.print("\t");
Serial.print(read);
Serial.print("\t");
Serial.println(_attitudeCmd);
//Serial.print("\t");
//Serial.print(CLAMP(15,abs(_attitudeCmd),175));
//Serial.print("\t");
//Serial.print(eps);
//Serial.print("\t");
//Serial.print(intEps);
//Serial.print("\t");
//Serial.println(millis());
#endif
			if(_attitudeCmd>=0) servoAttitude.write(_attitudeCmd);
			else servoAttitude.write(_attitudeCmd);

			servoAttitude.write(CLAMP(10,_attitudeCmd,175));
		}
		else {//to avoid problems due to long loop
			  time_prev_asser=millis();
			  intEps=0;//<=>resets the integral term
			  servoAttitude.write(CLAMP(10,_attitudeCmd,175));
			  readInertial();
			}
	}
}
