/*
this library contains the different functions useful for the motor and its control
*/

#include "lib_attitude.h"

//defines
#define ATTITUDE_ASSER_PERIOD 20 // milliseconds

#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

//globals
int _attitudeCon;
int _pinServo;

//initializes the PINs for motor speed and direction
//REQUIRES : odoinitHard()
void servoInitHard(int pinservo){
	_pinServo=pinservo;
	servoAttitude.attach(_pinServo);

}

#define KP  0// >>2
#define KI  0// >>2

void attitudeAsser(){

#ifdef DEBUG_ATTITUDE
Serial.println("Attitude");
#endif


//    unsigned long int time=millis();
//    static int intEps=0;
//    static unsigned long time_prev_asser=millis();
//    static int _attitudeCmd=0;
//	int eps;
//
//
//	// asservissement attitude
//	if((time-time_prev_asser)>=ATTITUDE_ASSER_PERIOD) {
//		if ( (time-time_prev_asser) < ATTITUDE_ASSER_PERIOD+ATTITUDE_ASSER_PERIOD/2 ){
//			time_prev_asser = time_prev_asser + ATTITUDE_ASSER_PERIOD;
//			//compute error (epsilon)
//			int read=MaSuperFonctionPourLire()////////////////////////////////////////////////////////////////////
//			eps = _attitudeCon - read;
//
//			//compute error integral
//			intEps= CLAMP( -(64<<4) ,intEps+eps, (64<<4));
//			//compute command
//			if(_motCon[i]==0){
//			_motCmd[i]=0;
//			}
//			else{
//				_motCmd[i]=  ((KP*eps)>>2) + ((KI*intEps[i])>>2);
//			}
//
//#ifdef DEBUG_ATTITUDE
//Serial.print(_motCon[i]);
//Serial.print("\t");
//Serial.print(read);
//Serial.print("\t");
//Serial.print(_motCmd[i]);
//Serial.print("\t");
//Serial.print(CLAMP(0,abs(_motCmd[i]),254));
//Serial.print("\t");
//Serial.print(eps);
//Serial.print("\t");
//Serial.print(intEps[i]);
//Serial.print("\t");
//Serial.println(millis());
//#endif
//			if(_motCmd[i]>=0) digitalWrite(_motPinDir[i],LOW);
//			else digitalWrite(_motPinDir[i],HIGH);
//
//			analogWrite(_motPinPWM[i],CLAMP(0,abs(_motCmd[i]),254));
//		}
//		else {//to avoid problems due to long loop
//			  time_prev_asser[0]=millis();
//			  intEps[0]=0;//<=>resets the integral term
//			  analogWrite(_motPinPWM[i], CLAMP(0,abs(_motCmd[i]),254));
//			  odoRead(i);
//			}
//	}
}
