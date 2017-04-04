

#include "Arduino.h"
#include "../tools.h"
#include "../params.h"
#include "state_pause.h"
#include "lib_radar.h"
#include "lib_move.h"
#include "state_dead.h"
#include "state_funny_action.h"
#include "lib_us.h"

sState *pausePrevState;
sState* testPause(){
    static unsigned long lastSeen;
    if(radarIntrusion()) lastSeen=millis();
    if( (millis()-lastSeen)>= RADAR_SAFETY_TIME ) return pausePrevState;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sDead;
#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif
    return 0;
}


void initPause(sState *prev){
    pausePrevState=prev;
    move(0,0);
#ifdef DEBUG
    Serial.println(F("debut pause"));
#endif
}

void deinitPause(sState *next){
#ifdef DEBUG
    Serial.println(F("fin pause"));
#endif
}

sState sPause={
    BIT(E_MOTOR) | BIT(E_RADAR),
    &initPause,
    &deinitPause,
    &testPause
};
