/*
 * ai.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: Sebastien Malissard
 */

#include <a_star_tools.h>
#include <ai.h>
#include <ai_tools.h>

extern "C"{
#include <string.h>
#include <stdlib.h>
#include <millis.h>
}

#include <obj_tools.h>
#include <clap.h>
#include <spot.h>
#include <obj.h>
#include <tools.h>
#include "communications.h"

#ifndef HOLONOMIC
#error "HOLONOMIC must be defined"
#endif

using namespace std;

std::vector<Obj*> listObj;

void colissionDetection(){
    Point2D<float> ptPr = statuses.getLastPosXY(ELT_PRIMARY);
    float anglePr = statuses.getLastOrient(ELT_PRIMARY);
    Point2D<float> ptAPr = statuses.getLastPosXY(ELT_ADV_PRIMARY);
    Point2D<float> ptASc = statuses.getLastPosXY(ELT_ADV_SECONDARY);
    sNum_t d, dot;
    int contact = 0;

    d = ptPr.distanceTo(ptAPr);
    Vector2D<float> v1(cos(anglePr), sin(anglePr)), v2(ptPr, ptAPr);
    dot = v1*v2;

    if (d < 50 && dot > 0.6 * d) {
        logs << INFO << "CONTACT PRIM!!!!!!!!!!!!!!!!!!!!!!!!!"; // TODO
        contact = 1;
    }

    d = ptPr.distanceTo(ptASc);
    Vector2D<float> v3(ptPr, ptASc);
    dot = v1 * v3;

    if (d < 40 && dot > 0.6 * d) {
        logs << INFO << "CONTACT SEC!!!!!!!!!!!!!!!!!!!!!!!!!"; // TODO
        contact = 1;
    }

    if (contact) {
        path.stopRobot();
    }

}

void initObjective(eColor_t color){
    if(color == YELLOW){
        listObj.push_back(new Clap(0));
        listObj.push_back(new Clap(2));
        listObj.push_back(new Clap(4));
    }
    else if(color == GREEN){
        listObj.push_back(new Clap(1));
        listObj.push_back(new Clap(3));
        listObj.push_back(new Clap(5));
    }
    else
        logs << ERR << "Color ???";

    for(unsigned int i = 0 ; i < 8 ; i++)
        listObj.push_back(new Spot(i, color));
}


int stepAI() {
    return 1;
}

int initAI() {

    //Objectives are initialized in the loop step, because here we don't known the color selected.

    return 0;
}
