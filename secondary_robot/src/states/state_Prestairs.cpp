/*
 * state_Prestairs.cpp
 *
 *  Created on: 2015 mars 18
 *      Author: Guilhem
 */


#include "Arduino.h"
#include "state_Prestairs.h"
#include "Arduino.h"
#include "state_types.h"
#include "state_stairs.h"
#include "lib_move.h"
#include "lib_attitude.h"
#include "../tools.h"
#include "../params.h"
#include "MPU_6050.h"

unsigned long timeStartPreStairs;
#ifdef ATTITUDE
	#define TIME_PRESTAIRS 2000
#else
	#define TIME_PRESTAIRS 2000
#endif
#define ANGLE_TO_READ 1
#define ATTITUDE_STAIRS_STARTED 10
#define  ANGLE_INCREMENT 19

sState* testPrestairs()
    {

#ifndef NO_ATTITUDE_BEFORE_STAIRS
        if (millis()-timeStartPreStairs > TIME_PRESTAIRS){
        	return &sStairs;
        }
       return NULL;
#else
        static int nb_coups=0;
		static int angle_cmd = 160;
		static int touchStairs = 0;
		static unsigned long timetouchStairs = 0;
		static int lasttimeincr;
		static unsigned long timeServo = 0;

       int angle = readInertial(ANGLE_TO_READ);
       Serial.print("Angle:    ");
       Serial.println(angle);
       if (abs(angle) > ATTITUDE_STAIRS_STARTED)
       {
    	   touchStairs = 1;
       }

       if(touchStairs)
       {
    	   move(0,0);
    	   if(timetouchStairs == 0){
    	       		   timetouchStairs = millis();
    	       		   lasttimeincr = timetouchStairs;
    	   }
    	   if(millis()-lasttimeincr > 1000  && nb_coups < 3){
    		   lasttimeincr = millis();
    		   angle_cmd -= ANGLE_INCREMENT;
    		   nb_coups ++;
    		   servoAttitude.write(angle_cmd);
    	   }

    	   if(/*millis()-timetouchStairs > 500  && */nb_coups >= 3)
    	   {
    		   Serial.println("Attitude Activated");
			   if(timeServo == 0){
				   timeServo = millis();
				   sPrestairs.flag = sPrestairs.flag|BIT(E_ATTITUDE);
			   }

			   if(millis()-timeServo > TIME_PRESTAIRS){
				   return &sStairs;
			   }

    	   }
       }

       return NULL;
#endif
    }

void initPrestairs(sState *prev)
    {
        timeStartPreStairs = millis();
#ifdef NO_ATTITUDE_BEFORE_STAIRS
        move(5,0);
#else
        move(0,0);
#endif
        Serial.print("PRESTAIRS Start");
    }

void deinitPrestairs(sState *next)
    {
		Serial.print("PRESTAIRS end");
    }

sState sPrestairs={
#ifdef NO_ATTITUDE_BEFORE_STAIRS
		BIT(E_MOTOR),
#else
		BIT(E_ATTITUDE)|BIT(E_MOTOR),
#endif
        &initPrestairs,
        &deinitPrestairs,
        &testPrestairs
};

