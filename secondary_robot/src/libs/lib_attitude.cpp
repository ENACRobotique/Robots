/*
this library contains the different functions useful for the motor and its control
*/

#include "lib_attitude.h"
#include "MPU_6050.h"
#include "state_types.h"
extern "C" {
#include "median_filter.h"
}

//defines
#define ATTITUDE_ASSER_PERIOD 40 // milliseconds
#define MAX_ANGLE 15
#define MIN_ANGLE 176
#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

//globals
int _attitudeCon = 0;	// >0 : incliné vers l'avant.
int _attitudeCmd=0;
int _pinServo;
Servo servoAttitude;
median_t mf;

//initializes the PINs for motor speed and direction
//REQUIRES : initInertial()
void servoInitHard(int pinservo){
	_pinServo=pinservo;
	servoAttitude.attach(_pinServo);
	servoAttitude.write(MIN_ANGLE);
	mf_init(&mf, 8, 180);
}

#define KP  1// >>2
#define KI  1// >>5
#define KD  0
#define ANGLE_STAIRS 17
#define CORR1 (((-(ANGLE_STAIRS-48)*180)<<2)/48)
#define CORR2 ((-180<<2)/48)

void attitudeAsser(){

    unsigned long int time=millis();
    static int intEps=-768;		//-768 = ((180<<4)/KI)/COOR2	correction par le terme intégral de l'offset
    static int prevEps = 0;
    static unsigned long time_prev_asser=millis();
	int eps;
	int derEps;


	// asservissement attitude
	if((time-time_prev_asser)>=ATTITUDE_ASSER_PERIOD) {
		if ( (time-time_prev_asser) < ATTITUDE_ASSER_PERIOD+ATTITUDE_ASSER_PERIOD/2 ){
			time_prev_asser = time_prev_asser + ATTITUDE_ASSER_PERIOD;
			//compute error (epsilon)
			int read= -readInertial(X_ANGLE);
			eps = _attitudeCon - read;
			int correct = CORR1 + CORR2 * _attitudeCon;

			//compute error derivate
			derEps = CLAMP( -(64<<2) ,eps - prevEps, (64<<2));
			prevEps = eps;

			//compute error integral
			intEps= CLAMP( -(64<<3) ,intEps+eps, (64<<3));
			//compute command
//			if(_attitudeCon==0){
//				_attitudeCmd=0;
//			}
//			else{
				_attitudeCmd = ((((KP*eps)>>3) + ((KI*intEps)>>4) + ((KD*derEps)>>2))*CORR2 + correct)>>2;
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
			mf_update(&mf, _attitudeCmd);

			servoAttitude.write(CLAMP(MAX_ANGLE,mf_get(&mf),MIN_ANGLE));
		}
		else {//to avoid problems due to long loop
			  time_prev_asser=millis();
			  intEps=0;//<=>resets the integral term
			  servoAttitude.write(CLAMP(MAX_ANGLE,_attitudeCmd,MIN_ANGLE));
			  readInertial(X_ANGLE);
			}
	}
}
