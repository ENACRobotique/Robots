/*
 * statuses.cpp
 *
 *  Created on: 21 févr. 2015
 *      Author: seb
 */

#include <main_ai_tools/statuses.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "ai_types.h"
#include "tools.h"
#include "time_tools.h"


Statuses::Statuses() {


}

Statuses::~Statuses() {
    // TODO Auto-generated destructor stub
}

void Statuses::maintenace(){
    // TODO delete old items of the lists (for ex. older than 5seconds)
    // TODO compare but need synchronized time (as well as in obj_tim_tools.c, I'm going to do it... and come back here later)

    //        if(millis() - prevGetPos > 200){
    //            sGenericPos *p;
    //
    //            prevGetPos = millis();
    //
    //            p = getLastPGPosition(ELT_ADV_PRIMARY);
    //            printf("ADV_PRIMARY %p\n", p);
    //
    //            p = getLastPGPosition(ELT_ADV_SEC);
    //            printf("ADV_SEC %p\n", p);
    //
    //            p = getLastPGPosition(ELT_PRIMARY);
    //            printf("PRIMARY %p\n", p);
    //
    //            p = getLastPGPosition(ELT_SECONDARY);
    //            printf("SECONDARY %p\n", p);
    //        }
}

int Statuses::receivedNewStatus(sGenericStatus &status){

    if(status.id >= NUM_E_ELEMENT){
        cerr << "[ERROR] [statuses.cpp] Unknown status id" << endl;
        return -1;
    }
    logs << INFO_V(E_V3) << "New status : " << status.pos.x << ", " << status.pos.y << ", " << status.pos.theta * 180 / M_PI;

    _list[status.id].push_back(status); //TODO sort by date

    if(status.id == ELT_PRIMARY)
        logs.putNewPos(status.pos.x, status.pos.y, status.pos.theta);

    return 1;
}

sGenericStatus& Statuses::getLastStatus(eElement el, frame_t fr){
    if(!_list[el].empty()){
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

Point2D<float> Statuses::getLastPosXY(eElement el){
    sGenericStatus& status = getLastStatus(el);

    return {status.pos.x, status.pos.y};
}

float Statuses::getLastOrient(eElement el){
    sGenericStatus& status = getLastStatus(el);

    return status.pos.theta;
}

float Statuses::getLastSpeed(eElement el){

    if(_list[el].size() >=2){
        sGenericStatus& status1 = getLastStatus(el);
        Point2D<float> pt1 = {status1.pos.x, status1.pos.y};

        sGenericStatus& status2 =_list[el][_list[el].size() - 2];
        Point2D<float> pt2 = {status2.pos.x, status2.pos.y};

        float dist = pt1.distanceTo(pt2);

        return dist/(status1.date - status2.date)*1000000;
    }


    return 0;
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
 * Update the new position on monitoring
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
