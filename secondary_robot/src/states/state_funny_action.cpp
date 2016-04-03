
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "../tools.h"
#include "../params.h"
#include "state_wait.h"
#include "state_funny_action.h"

Servo parasol_servo;

sState *TestFunnyAction(){
	// do the funny action
	if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sWait;
	return 0;
}

void initFunnyAction(sState *prev){
	move(0,0);
	parasol_servo.write(174);
#ifdef DEBUG
	Serial.println(F("Je fais la funny action!"));
#endif
}


void deinitFunnyAction(sState *next){
#ifdef DEBUG
    Serial.println(F("Fin funny action"));
#endif
}

sState sFunnyAction={
		BIT(E_MOTOR),
		&initFunnyAction,
		&deinitFunnyAction,
		&TestFunnyAction,
};

