/*
this library contains the different functions useful for the motor and its control
*/

#include "lib_heading.h"
#include "MPU_6050.h"
#include "lib_move.h"

//defines
#define HEADING_ASSER_PERIOD 80 // milliseconds
#define ANGLE_TO_ASSERV Z_ANGLE

#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

//globals
int _speedCon = 0;
int _OmegaCmd=0;
int _headingCon = 0;

//TODO : Check if lib_motor is initialized !
//initializes the PINs for motor speed and direction
//REQUIRES : initInertial()

#define KP  3// >>2
#define KI  0// >>8
#define KD  0

int headingGetCurrent()
{
	return readInertial(ANGLE_TO_ASSERV);
}

void headingAsser(){
    unsigned long int time=millis();
    static int intEps=0;
    static int prevEps = 0;
    static unsigned long time_prev_asser=millis();
	int eps;
	int derEps;


	// asservissement attitude
	if((time-time_prev_asser)>=HEADING_ASSER_PERIOD) {
		if ( (time-time_prev_asser) < HEADING_ASSER_PERIOD+HEADING_ASSER_PERIOD/2 ){
			time_prev_asser = time_prev_asser + HEADING_ASSER_PERIOD;
			//compute error (epsilon)
			int read=readInertial(ANGLE_TO_ASSERV);
			eps = _headingCon - read;

			//compute error derivate
			derEps = CLAMP( -(64<<2) ,eps - prevEps, (64<<2));
			prevEps = eps;

			//compute error integral
			intEps= CLAMP( -(64<<2) ,intEps+eps, (64<<2));
			//compute command
			_OmegaCmd = ((KP*eps)>>2) + ((KI*intEps)>>3) + (KD*derEps);
#ifdef DEBUG_HEADING
Serial.print(_headingCon);
Serial.print("\t");
Serial.print(read);
Serial.print("\t");
Serial.println(_OmegaCmd);
#endif
			move(_speedCon, _OmegaCmd);
		}
		else {//to avoid problems due to long loop
			  time_prev_asser=millis();
			  intEps=0;//<=>resets the integral term
			  move(_speedCon, _OmegaCmd);
			  readInertial(ANGLE_TO_ASSERV);
			}
	}
}
