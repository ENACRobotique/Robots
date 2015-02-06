/*
 * state_Manualdrive.cpp
 *
 *  Created on: 2015 janvier 28
 *      Author: Fabien
 */


#include "Arduino.h"
#include "state_Manualdrive.h"
#include "../tools.h"
#include "lib_move.h"

sState* testManualdrive(){
	static long time = millis();
	if ((millis() - time) > 200){
        int speed = analogRead(1);
        int omega = analogRead(0);
        speed = map(speed,0,1023,-60,60);
        omega = -map(omega,0,1023,-20,20);
        move(speed,omega);
        time = millis();
#ifdef DEBUG_MANUAL
        Serial.print(time);
        Serial.print("\t");
        Serial.print(speed);
        Serial.print("\t");
        Serial.println(omega);
#endif
	}
        return NULL;
}

void initManualdrive(sState *prev)
    {
        // Nothing
    }

void deinitManualdrive(sState *next)
    {
        move(0,0);
    }

sState sManualdrive={
        BIT(E_MOTOR),
        &initManualdrive,
        &deinitManualdrive,
        &testManualdrive
};

