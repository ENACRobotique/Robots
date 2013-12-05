/*
 * state_tirette.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */


#include "Arduino.h"
#include "state_tirette.h"
#include "state_goline.h"
#include "../params.h"
#include "../tools.h"

#include "lib_move.h"
#include "sharp_2d120x.h"

/* State : tirette, first state of all, waits until the tirette is pulled
 *
 * tirette pulled -> next state, according to the "start side swich"
 *
 */
sState* testTirette(){
    static unsigned long prevIn=0;  //last time the tirette was seen "in"
    if (digitalRead(PIN_TIRETTE)==TIRETTE_IN) prevIn=millis();
    if ( ( millis() - prevIn) > DEBOUNCE_DELAY) {
        if (digitalRead(PIN_COLOR)==COLOR_RED)return &sGolineRed;
        else return &sGolineRed; //a remplacer par &sGolineYellow
    }
    return 0;
}

void initTirette(sState *prev){
    move(0,DIR_SERVO_START);
    armServoLeft.write(ARM_LEFT_UP);
    armServoRight.write(ARM_RIGHT_UP);
}

void deinitTirette(sState *next){
    _matchStart=millis();
#ifdef DEBUG
    Serial.println("fin tirette");
#endif

}
sState sTirette={
        BIT(E_MOTOR),
        &initTirette,
        &deinitTirette,
        &testTirette
};



int EnemyDetection()
	{
	if(raw2dist120x(analogRead(PIN_SHARP_FRONT))< 16*ENEMY_SAFETY_DST) return 1;
	Serial.println(raw2dist120x(analogRead(PIN_SHARP_FRONT))/16);
	return 0;
	}
