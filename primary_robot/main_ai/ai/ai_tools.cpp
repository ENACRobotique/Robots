/*
 * obj_fct.c
 *
 *  Created on: 29 mars 2014
 *      Author: seb
 */


#include <ai_tools.h>
#include <GeometryTools.h>
#include <obj_tools.h>
#include "ai.h"
#include "botNet_core.h"

extern "C"{
#include "millis.h"
}

void printListObj() {
    for(Obj* i : listObj)
        i->print();
}

void printObsActive() {
    int i;
    logs << INFO << "List of obs[i].active :\n";
    for (i = 0; i < N; i++)
        logs << "obs[" << i << "].active=" << obs[i].active << "\n";
}


int test_in_obs(Point2D<float> p) { //retourne le numéros de l'obstable si la position est a l'interieur de celui ci
    //FIXME si le robot dans plusieurs obstable
    for (int i = 1; i < N - 1; i++) {
        if (obs[i].active == 0)
            continue;
        Point2D<float> c(obs[i].c.x, obs[i].c.y);
        if (c.distanceTo(p) < obs[i].r) {
            //printf("Le robot est dans l'obstacle n=%i, robs=%f, xobs=%f, yobs=%f, currentpos x=%f, y=%f\n",i,obs[i].r,obs[i].c.x,obs[i].c.y, _current_pos.x, _current_pos.y);
            return i;
        }
    }
    return 0;
}

int test_tirette(void) {
#if SIMU
    return 1;
#endif
    return starting_cord;
}

void startColor(void) {

#if SIMU
   // color = COLOR_SIMU; //FIXME
#else
    static int state = 0;
    sMsg msgOut;

    switch(state) {
        case 0 :
        if(mode_switch == 0) { //Red color
            state = 1;
            color = 0;

            msgOut.header.destAddr = ADDRI1_MAIN_IO;
            msgOut.header.type = E_IHM_STATUS;
            msgOut.header.size = 2 + 1*sizeof(*msgOut.payload.ihmStatus.states);

            msgOut.payload.ihmStatus.nb_states = 1;
            msgOut.payload.ihmStatus.states[0].id = IHM_LED;
            msgOut.payload.ihmStatus.states[0].state = 1;

            bn_sendRetry(&msgOut, MAX_RETRIES);
        }
        break;
        case 1 :

        if(mode_switch == 1) {
            state = 2;
        }
        break;
        case 2 :
        if(mode_switch == 0) { //Yellow color
            state = 3;
            color = 1;

            msgOut.header.destAddr = ADDRI1_MAIN_IO;
            msgOut.header.type = E_IHM_STATUS;
            msgOut.header.size = 2 + 1*sizeof(*msgOut.payload.ihmStatus.states);

            msgOut.payload.ihmStatus.nb_states = 1;
            msgOut.payload.ihmStatus.states[0].id = IHM_LED;
            msgOut.payload.ihmStatus.states[0].state = 2;

            bn_sendRetry(&msgOut, MAX_RETRIES);
        }
        break;
        case 3 :
        if(mode_switch == 1) {
            state = 0;
        }
        break;
    }
#endif
}


void simuSecondary(void) { //TODO if a other robot on trajectory

}


void posPrimary(void) { //FIXME permet de deplacer les objects mobile en cas de contact
    int i;
    Point2D<float> pt;
    Point2D<float> _current_pos = statuses.getLastPosXY(ELT_PRIMARY);

    if (((i = test_in_obs(_current_pos)) != 0)) {
        if (obs[i].moved == 1) {
            pt = {obs[0].c.x, obs[0].c.y};
            Circle2D<float> cir(obs[i].c.x, obs[i].c.y, obs[i].r);
            pt = cir.projecte(pt);
            Vector2D<float> v(pt, {obs[0].c.x, obs[0].c.y});

            obs[i].c.x += v.x;
            obs[i].c.y += v.y;

            obs_updated[i]++;
        }
        Point2D<float> p(_current_pos.x, _current_pos.y);
        Circle2D<float> c(obs[i].c.x, obs[i].c.y, obs[i].r);
        p = c.projecte(p);
        _current_pos = {p.x, p.y};
        if (sqrt(pow(_current_pos.x - obs[0].c.x, 2) + pow(_current_pos.y - obs[0].c.y, 2) < 2)) {

            obs[0].c = {_current_pos.x, _current_pos.y};
            obs_updated[0]++;
        }
        else {
            _current_pos = {obs[0].c.x, obs[0].c.y};
            obs_updated[0]++;
        }
    }
    //if non holmic
    float theta_robot = statuses.getLastOrient(ELT_ADV_PRIMARY);
    Point2D<float> p = {obs[0].c.x, obs[0].c.y};
    updateNoHaftTurn(theta_robot * 180 / M_PI, pt);
    obs_updated[N - 5]++;
    obs_updated[N - 6]++;
    obs_updated[N - 7]++;
    //end if
}


//TODO Optimisation des deplacements du robot algorithme arbre recouvrant


