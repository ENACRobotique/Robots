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
    return 0;
}
void initFunny(sState *prev){
    armServoLeft.write(ARM_LEFT_DOWN);
    armServoRight.write(ARM_RIGHT_DOWN);
    digitalWrite(PIN_FUNNY,HIGH);
    move(0,0);
}
void deinitFunny(sState *next){
    digitalWrite(PIN_FUNNY,LOW);
}
void funny(){
    //todo : wave your arms!!!
}
sState sFunny={
    BIT(E_FUNNY)|BIT(E_MOTOR),
    &initFunny,
    &deinitFunny,
    &testFunny
};
