
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
#include "lib_trajectory.h"

#define FACTOR_HEADING_ASSERV 1.5

int _backFromPause = 0;

int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis){
	static int teta0 = headingGetCon();
	int dt = millis() - *prev_millis -*pausetime;    //time since start of the current traj element
	int teta = tab[*i].teta*min((dt*FACTOR_HEADING_ASSERV/tab[*i].duration),1);
	headingSetCon( teta + teta0);

    if (!(*prev_millis)){
    	*prev_millis=millis();
    	speedSetCon(tab[*i].speed);
        //move(tab[*i].speed,tab[*i].teta);
        teta0 = headingGetCon();
    }

    if(_backFromPause){
    	_backFromPause = 0;
    	speedSetCon(tab[*i].speed);
    	//move(tab[*i].speed,tab[*i].teta);

    }

    if ( (millis()-*prev_millis-*pausetime)>tab[*i].duration ) {
        (*i)++;
        *prev_millis=millis();
        *pausetime=0;
        teta0 = headingGetCon();
        speedSetCon(tab[*i].speed);
        //move(tab[*i].speed,tab[*i].teta);
    }
    if ( tab[*i].teta==0 && tab[*i].duration==0 && tab[*i].speed==0) {
        *i=0;
        *prev_millis=0;
        return 1;
    }

    return 0;
}
