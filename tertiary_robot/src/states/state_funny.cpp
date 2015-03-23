/*
 * state_funny.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */


#include "Arduino.h"
#include "../params.h"
#include "../tools.h"
#include "state_dead.h"
#include "state_funny.h"
#include "lib_move.h"



sState* testFunny(){
    if ((millis()-_matchStart) > TIME_FUNNY_STOP ) return &sDead;


    if ((millis()-_matchStart) > TIME_MATCH_STOP)
    	Serial.println("Go funny !");//do what you want to
    return 0;
}
void initFunny(sState *prev)
	{
	move(0,0);
	launcherServoUp.write(40);

	launcherServoDown.write(30);


	}
void deinitFunny(sState *next)
	{

	}

sState sFunny={
    BIT(E_MOTOR),
    &initFunny,
    &deinitFunny,
    &testFunny
};
