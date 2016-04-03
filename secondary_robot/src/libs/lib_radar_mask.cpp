
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_line.h"
#include "lib_wall.h"
#include "lib_attitude.h"
#include "lib_heading.h"
#include "../tools.h"
#include "../params.h"
#include "state_pause.h"
#include "lib_radar_mask.h"

int _backFromPauseRadar = 0;

int periodicProgRadarLimit(radarElem tab[], unsigned long *pausetime, int *i, unsigned long *prev_millis){
	static uint16_t limit_mem = 40;
	if (!(*prev_millis)){
		    	*prev_millis=millis();
		    	Serial.print("total time:  ");
		    	Serial.println(tab[*i].duration);
		    }

	long dt = millis() - *prev_millis -*pausetime;    //time since start of the current traj element
	long duration = long(tab[*i].duration);
	uint16_t limit = ((tab[*i].limit_end-tab[*i].limit_start)*dt)/duration + tab[*i].limit_start;
	uint16_t limits[RAD_NB_PTS]={limit,limit};
	radarSetLim(limits);


	if(limit != limit_mem){
#ifdef DEBUG_RADAR
		Serial.print("start: ");
		Serial.print(tab[*i].limit_start);
		Serial.print("   end: ");
		Serial.print(tab[*i].limit_end);
		Serial.print("   limit:  ");
		Serial.print(limit);
		Serial.print("   time:  ");
		Serial.println(dt);
#endif
		limit_mem = limit;
	}


    if ( (millis()-*prev_millis-*pausetime)>tab[*i].duration ) {
        (*i)++;
        *prev_millis=millis();
        *pausetime=0;
        Serial.print("total time:  ");
        	Serial.println(tab[*i].duration);
    }

    if ( tab[*i].limit_start==0 && tab[*i].duration==0 && tab[*i].limit_end==0) {
        *i=0;
        *prev_millis=0;
        return 1;
    }

    return 0;
}

