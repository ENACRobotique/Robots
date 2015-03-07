/*
this library contains the different functions useful for the motor and its control
*/

#include "lib_attitude.h"
#include "MPU_6050.h"
#include "../params.h"

//defines
#define ATTITUDE_ASSER_PERIOD 20 // milliseconds
#define ANGLE_TO_ASSERV X_ANGLE
#define MAX_ANGLE 15
#define MIN_ANGLE 180
#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

//globals
int _attitudeCon = -8;
int _attitudeCmd=0;
int _pinServo;
Servo servoAttitude;

//initializes the PINs for motor speed and direction
//REQUIRES : odoinitHard()
void servoInitHard(int pinservo){
	_pinServo=pinservo;
	servoAttitude.attach(_pinServo);
	servoAttitude.write(MIN_ANGLE);
}

#define KP  4// >>2
#define KI  2// >>8

void attitudeAsser(){

    unsigned long int time=millis();
    static int intEps=256;
    static unsigned long time_prev_asser=millis();
	int eps;


	// asservissement attitude
	if((time-time_prev_asser)>=ATTITUDE_ASSER_PERIOD) {
		if ( (time-time_prev_asser) < ATTITUDE_ASSER_PERIOD+ATTITUDE_ASSER_PERIOD/2 ){
			time_prev_asser = time_prev_asser + ATTITUDE_ASSER_PERIOD;
			//compute error (epsilon)
			int read=-readInertial(ANGLE_TO_ASSERV);
			eps = _attitudeCon - read;

			//compute error integral
			intEps= CLAMP( -(64<<2) ,intEps+eps, (64<<2));
			//compute command
//			if(_attitudeCon==0){
//				_attitudeCmd=0;
//			}
//			else{
				_attitudeCmd = ((KP*eps)>>2) + ((KI*intEps)>>3);
//			}

#ifdef DEBUG_ATTITUDE
Serial.print(0.01);
Serial.print("\t");
Serial.print(_attitudeCon);
Serial.print("\t");
Serial.print(read);
Serial.print("\t");
Serial.print(90+_attitudeCmd);
//Serial.print("\t");
//Serial.print(CLAMP(15,abs(_attitudeCmd),175));
Serial.print("\t");
Serial.print(eps);
Serial.print("\t");
Serial.print(intEps);
//Serial.print("\t");
//Serial.print(millis());
Serial.println();
#endif
			//if(_attitudeCmd>=0) servoAttitude.write(180-_attitudeCmd);
			//else servoAttitude.write(180-_attitudeCmd);

			servoAttitude.write(CLAMP(MAX_ANGLE,90+_attitudeCmd,MIN_ANGLE));
		}
		else {//to avoid problems due to long loop
			  time_prev_asser=millis();
			  intEps=0;//<=>resets the integral term
			  servoAttitude.write(CLAMP(MAX_ANGLE,90+_attitudeCmd,MIN_ANGLE));
			  readInertial(ANGLE_TO_ASSERV);
			}
	}
}
