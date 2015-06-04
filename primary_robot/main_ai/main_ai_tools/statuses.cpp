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
#include "ai_tools.h"
#include "tools.h"
#include "time_tools.h"
extern "C"{
#include "string.h"
}


Statuses::Statuses() {
    for(unsigned int i = 0 ; i < NUM_E_ELEMENT; i++)
        reset[i] = false;

    reset[ELT_PRIMARY] = true;
    pt[ELT_PRIMARY] = {0, 0};


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

int Statuses::receivedNewStatus(sGenericPosStatus& status){

    logs << INFO_V(E_V3) << "New status (id:)" << status.id << " : (" << status.pos.x << ", " << status.pos.y << ", " << status.pos.theta * 180 / M_PI << "°)";

    if(status.id >= NUM_E_ELEMENT){
        logs << ERR << "Unknown status id=" << status.id;
        return -1;
    }
    if(!status.date){
        logs << ERR << "Date null";
        return -1;
    }
    if(status.pos.frame >= NUM_FRAME){
        logs << ERR << "Unknown frame=" << status.pos.frame;
        return -1;
    }
    if(fabs(status.pos.x) > 500. || fabs(status.pos.y) > 500.){
        logs << ERR << "Incoherent position x=" << status.pos.x << " y=" << status.pos.y;
        return -1;
    }
    if(fabs(status.pos.theta) > 10*M_PI){
        logs << ERR << "Incoherent angle theta=" << status.pos.theta*180/M_PI;
        return -1;
    }

    if(status.pos.frame == FRAME_PRIMARY){
        if(!_list[ELT_PRIMARY].empty()){
            sGenericPosStatus statusRet = status;
            sGenericPosStatus statusPrim = _list[eElement::ELT_PRIMARY].back();

            //FIXME get coherent primary date

            fromPRPG2PG(&status.pos, &status.pos_u, &statusPrim.pos,
                    &statusPrim.pos_u, &statusRet.pos, &statusRet.pos_u);

            status = statusRet;
        }
        else
            return -2;
    }

    if(status.pos.frame == FRAME_PLAYGROUND)
        _list[status.id].push_back(status); //TODO short by date
    else
        logs << ERR << "Position not save frame not in playground";

    if(status.id == ELT_PRIMARY)
        logs.putNewPos(status.pos.x, status.pos.y, status.pos.theta, status.pos_u.a_var, status.pos_u.b_var, status.pos_u.a_angle, status.pos_u.theta_var);

    return 1;
}

void Statuses::posSend(eElement el, Point2D<float>& p){
    reset[el] = true;
    pt[el] = p;
}

sGenericPosStatus& Statuses::getLastStatus(eElement el, frame_t fr){
    static sGenericPosStatus status;
    status.id = el;
    status.date = 0;

    if(!_list[el].empty()){
        if(reset[el]){
            logs << WAR << "Waiting positing for this element : " << el;
            Point2D<float> pos(_list[el].back().pos.x, _list[el].back().pos.y);
            if(pt[el].distanceTo(pos) < 3.){
                reset[el] = false;
            }
            else{
                return status;
            }

        }
        if(_list[el].back().pos.frame == fr){
            return _list[el].back();
        }
        else{
            logs << ERR << "Bad reference";
        }
    }

    //TODO ask position

    return status;
}

Point2D<float> Statuses::getLastPosXY(eElement el){
    sGenericPosStatus& status = getLastStatus(el);

    if(status.date)
        return {status.pos.x, status.pos.y};

    return {0,0};
}

float Statuses::getLastOrient(eElement el){
    sGenericPosStatus& status = getLastStatus(el);

    return status.pos.theta;
}

float Statuses::getLastSpeed(eElement el){

    if(el == ELT_PRIMARY){
        if(_list[el].size() > 0) {
            sGenericPosStatus& status = getLastStatus(el);
            Vector2D<float> v{status.prop_status.spd.vx, status.prop_status.spd.vy};
            return v.norm();
        }
    }
    else if(_list[el].size() >=2){
        sGenericPosStatus& status1 = getLastStatus(el);
        Point2D<float> pt1 = {status1.pos.x, status1.pos.y};

        sGenericPosStatus& status2 =_list[el][_list[el].size() - 2];
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
        dstPAPG->frame = FRAME_PLAYGROUND;
    }

    // create uncertainty if asked to
    if (srcUPR && srcUPG && dstUPG) {
        // TODO compute full uncertainty
    }
}
