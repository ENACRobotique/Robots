
#include "Arduino.h"
#include "state_Recalage.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_line.h"
#include "lib_wall.h"
#include "lib_attitude.h"
#include "lib_heading.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_wait.h"
#include "lib_radar_mask.h"
#include "state_Peche.h"

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

