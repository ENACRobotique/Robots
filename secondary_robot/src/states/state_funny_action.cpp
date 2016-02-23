
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "../tools.h"
#include "../params.h"
#include "state_wait.h"

Servo parasol_servo;

sState *TestFunnyAction(){
	// do the funny action
	if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sWait;
	return 0;
}

void initFunnyAction(sState *prev){
	move(0,0);
	parasol_servo.attach(PIN_PARASOL);
	parasol_servo.write(170);
#ifdef DEBUG
	Serial.println("Je fais la funny action!");
#endif
}


void deinitFunnyAction(sState *next){
#ifdef DEBUG
    Serial.println("Fin funny action");
#endif
}

sState sFunnyAction={
		BIT(E_MOTOR),
		&initFunnyAction,
		&deinitFunnyAction,
		&TestFunnyAction,
};

