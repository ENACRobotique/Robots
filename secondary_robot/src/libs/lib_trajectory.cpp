
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
	int dt = millis() - *prev_millis;    //time since start of the current traj element
	int teta = tab[*i].omega*min(dt*FACTOR_HEADING_ASSERV,tab[*i].duration)/1000;
	tetaSetCon( teta + teta0);

    if (!(*prev_millis)){
    	*prev_millis=millis();
        move(tab[*i].speed,tab[*i].omega);
        teta0 = headingGetCon();
    }

    if(_backFromPause){
    	_backFromPause = 0;
    	move(tab[*i].speed,tab[*i].omega);

    }

    if ( (millis()-*prev_millis-*pausetime)>tab[*i].duration ) {
        (*i)++;
        *prev_millis=millis();
        *pausetime=0;
        teta0 = headingGetCon();
        move(tab[*i].speed,tab[*i].omega);
    }
    if ( tab[*i].omega==0 && tab[*i].duration==0 && tab[*i].speed==0) {
        *i=0;
        *prev_millis=0;
        return 1;
    }

    return 0;
}
