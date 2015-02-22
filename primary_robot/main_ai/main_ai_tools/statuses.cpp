/*
 * statuses.cpp
 *
 *  Created on: 21 févr. 2015
 *      Author: seb
 */

#include <main_ai_tools/statuses.h>
#include <iostream>
#include <cmath>

#include "ai_types.h"


Statuses::Statuses() {


}

Statuses::~Statuses() {
    // TODO Auto-generated destructor stub
}

void maintenace(){
    // TODO delete old items of the lists (for ex. older than 5seconds)
    // TODO compare but need synchronized time (as well as in obj_tim_tools.c, I'm going to do it... and come back here later)
}

int Statuses::receivedNewStatus(sGenericStatus &status){

    if(status.id < 0 || status.id >NUM_E_ELEMENT){
        cerr << "[ERROR] [statuses.ccp] Unknown status id" << endl;
        return -1;
    }

    _list[status.id].push_back(status); //TODO sort by date
    return 1;
}

sGenericStatus& Statuses::getLastStatus(eElement el, frame_t fr){
    if(!_list.empty()){
        if(_list[el].back().pos.frame == fr){
            return _list[el].back();
        }
        else{
            //TODO convert, used fromPRPG2PG
            return _list[el].back();
        }
    }
    //TODO ask position
    static sGenericStatus status;
    status.id = el;
    status.date = 0;

    return status;
}

sPt_t Statuses::getLastPosXY(eElement el){
    sGenericStatus status = getLastStatus(el);
    sPt_t point;

    point.x = status.pos.x;
    point.y = status.pos.y;

    return point;
}


/*
 * Convert a position define to a frame in an other frame
 */
void Statuses::fromPRPG2PG(s2DPosAtt *srcPAPR, s2DPAUncert *srcUPR, s2DPosAtt *srcPAPG, s2DPAUncert *srcUPG, s2DPosAtt *dstPAPG, s2DPAUncert *dstUPG) {
    float theta;

    // create position if asked to
    if (dstPAPG) {
        theta = srcPAPG->theta - M_PI / 2.;
        dstPAPG->x = cos(theta) * srcPAPR->x - sin(theta) * srcPAPR->y + srcPAPG->x;
        dstPAPG->y = sin(theta) * srcPAPR->x + cos(theta) * srcPAPR->y + srcPAPG->y;
        dstPAPG->theta = theta + srcPAPR->theta;
    //    dstPAPG->frame = FRAME_PLAYGROUND;
    }

    // create uncertainty if asked to
    if (srcUPR && srcUPG && dstUPG) {
        dstUPG->theta = srcUPR->theta + srcUPG->theta;
        // TODO compute full uncertainty
    }
}

/*
 * Udapte the new position on monitoring
 */
void Statuses::posUpdated(sGenericStatus &status) {
    if (status.id != ELT_PRIMARY) { //Only for element fix in obs such as robots
        obs[status.id].active = 1;
        obs[status.id].moved = 1;
        obs[status.id].c.x = status.pos.x;
        obs[status.id].c.y = status.pos.y;

        obs_updated[status.id]++;
    }
}
