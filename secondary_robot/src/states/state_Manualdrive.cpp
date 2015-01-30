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

sState* testManualdrive()
    {
        int speed = analogRead(0);
        int omega = analogRead(1);
        //speed = map(speed,0,1023,0,255);
        //omega = map(omega,0,1023,0,255);
        speed = max(speed,254);
        omega = max(omega,254);
        move(omega,speed);
#if DEBUG
        Serial.print("speed: ");
        Serial.print(speed);
        Serial.print("omega: ");
        Serial.print(omega);
#endif
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
        BIT(E_MOTOR),    ///a faire.......
        &initManualdrive,
        &deinitManualdrive,
        &testManualdrive
};

