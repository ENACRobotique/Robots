

#include "Arduino.h"
#include "../tools.h"
#include "../params.h"
#include "state_pause.h"
#include "state_funny.h"
#include "lib_radar.h"
#include "lib_move.h"
#include "state_tirette.h"

sState *pausePrevState;
sState* testPause(){
    static unsigned long lastSeen;
    if(EnemyDetection()) lastSeen=millis();
    if( (millis()-lastSeen)>= ENEMY_SAFETY_TIME ) return pausePrevState;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    return 0;
}


void initPause(sState *prev){
    pausePrevState=prev;
    move(0,0);
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
    BIT(E_MOTOR),
    &initPause,
    &deinitPause,
    &testPause
};
