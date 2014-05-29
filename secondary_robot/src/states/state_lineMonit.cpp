/*
 * state-lineMonit.cpp
 */

#include "Arduino.h"
#include "../params.h"
#include "../tools.h"
#include "state_types.h"
#include "state_funny.h"
#include "state_pause.h"
#include "lib_radar2.h"

unsigned long int saveTime=0,prev_Time=0;

sState* testLineMonit(){

	/*if( millis()-saveTime== 2000 || prev_Time + (millis()-saveTime)==2000 ) {//suivi de ligne de 2 sec et lancer les balles
		launcherServoUp.write(LAUNCHER_UP_POS_1);
		launcherServoDown.write(LAUNCHER_DOWN_POS_1);

		}*/

	//launcherServoUp.write(LAUNCHER_UP_POS_1);
			//launcherServoDown.write(LAUNCHER_DOWN_POS_1);

	Serial.println("je suis en boucle de ligne");

	//if(radarIntrusion()) return &sPause;
    return 0;


}
void initLineMonit(sState *prev){

	#ifdef DEBUG
		Serial.println("j'ai détecté la  ligne.....");
	#endif

		if (prev==&sPause)
		    	{
				#ifdef DEBUG
					Serial.println("retour d'une pause");
				#endif
					saveTime=millis();
		    	}
		else {

			saveTime=millis();
	     	}

}
void deinitLineMonit(sState *next){
	if (next==&sPause) {
			prev_Time= millis()- saveTime;
	           }
	else
	    	{
	        saveTime=0;
	    	}
}


sState sLineMonit={
	BIT(E_MOTOR)| BIT(E_LINE),
    &initLineMonit,
    &deinitLineMonit,
    &testLineMonit
};
