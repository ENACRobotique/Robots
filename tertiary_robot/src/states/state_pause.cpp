

#include "Arduino.h"
#include "../tools.h"
#include "params.h"
#include "state_pause.h"
#include "lib_radar.h"
#include "lib_move.h"
#include "state_tirette.h"
#include "state_dead.h"
#include "sharp_2d120x.h"

sState *pausePrevState;
sState* testPause(){
    static unsigned long lastSeen;
#ifndef SHARP_ONLY
    if(radarIntrusion()) lastSeen=millis();
#else
    if(sharpIntrusion()) lastSeen=millis();
#endif
    if( (millis()-lastSeen)>= RADAR_SAFETY_TIME ) return pausePrevState;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) {
    	emergencyStop();
    	return &sDead;
    }
    return 0;
}


void initPause(sState *prev){
    pausePrevState=prev;
    move(0,0);
    Serial.println("je suis en pause");
#ifdef DEBUG
    Serial.println("debut pause");
#endif
}

void deinitPause(sState *next){
#ifdef DEBUG
    Serial.println("fin pause");
#endif
}

sState sPause={
    BIT(E_MOTOR) | BIT(E_SHARP),
    &initPause,
    &deinitPause,
    &testPause
};
