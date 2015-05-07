/*
 * CapAI.cpp
 *
 *  Created on: 13 avr. 2015
 *      Author: seb
 */

#include "CapAI.h"
#include "CapIO.h"
#include "CapPropulsion.h"
#include "CapPosition.h"
#include "CapActuator.h"
#include "CapTeam.h"
#include "tools.h"
#include "ai_tools.h"
#include "a_star_tools.h"
#include "obj_tools.h"
#include "communications.h"
extern "C"{
#include "millis.h"
}
#include "clap.h"
#include "spot.h"
#include "cup.h"
#include "dropCup.h"
#include "environment.h"


int CapAI::loop(){
    static bool mode_obj = false;
    static int current_obj = -1;
    static Point2D<float> pt_select;
    static unsigned int last_time = 0;
    static unsigned int start_time = 0;
    static unsigned int lastDectectTime = 0;
    bool contact = false;

    CapPropulsion* capProp = dynamic_cast<CapPropulsion*> (robot->caps[eCap::PROP]);
    CapPosition* capPos = dynamic_cast<CapPosition*> (robot->caps[eCap::POS]);
    CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);
    CapActuator* capActuator = dynamic_cast<CapActuator*> (robot->caps[eCap::ACTUATOR]);

    start_time = capTeam->getStartGame();

    if (millis() - start_time > END_MATCH){
        logs << INFO << "SHUT_DOWN : time = " << (unsigned int) (millis() - start_time) / 1000;
        path.stopRobot(capProp->getPropType()==HOLO?true:false);
        return 0;
    }

    if(colissionDetection(robot->el, robot->env->obs) && ((millis() - lastDectectTime) > 500)){
        lastDectectTime = millis();
        path.stopRobot(true);
        last_time = 0;
        contact = true;

    }

    if (!mode_obj) {
        if(listObj.empty()) //Test if all objective have finished
            logs << INFO << "Objective list is empty";

        if (((millis() - last_time) > 1000) || contact){ //Calculation of the next objective
            last_time = millis();

            if ((current_obj = nextObj(start_time, listObj, robot->env->obs, robot->env->obs_updated ,(int) capPos->getIobs(), capProp->getPropType()==AXLE?true:false, capActuator->_act)) != -1) {
                pt_select = listObj[current_obj]->getDestPoint();
                logs << INFO << "Selected point is (" << pt_select.x << " ; " << pt_select.y << ")";

                if(capProp->getPropType() == AXLE){
                    /*      int k = listObj[current_obj]->_EP;
                    sObjPt_t ep = listObj[current_obj]->entryPoint(k);
                    updateEndTraj(ep.angleEP,  &ep.c, ep.radiusEP);
                    printEndTraj();*/
                    }
                sPath_t path_loc = listObj[current_obj]->getPath();
                path.addPath2(path_loc);
                path.sendRobot(capProp->getPropType()==HOLO?true:false, listObj[current_obj]->getDestPointOrient());
            }
        }

        if ( (pt_select.distanceTo(capPos->getLastPosXY()) < RESO_POS) && (current_obj != -1) ){ //Test if the robot is on the entry point selected
            mode_obj = true;
        }
    }else{
        if (metObj(current_obj, listObj, robot->env->obs, robot->env->obs_updated, capActuator->_act ) == 0){
            pt_select.x = -1;
            pt_select.y = -1;
            mode_obj = false;
        }
    }

    return 1;
}

void CapAI::initObjective(){
    CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);

    if(capTeam->getColor() == YELLOW){
        listObj.push_back(new Clap(0));
        listObj.push_back(new Clap(2));
        listObj.push_back(new Clap(4));
    }
    else if(capTeam->getColor() == GREEN){
        listObj.push_back(new Clap(1));
        listObj.push_back(new Clap(3));
        listObj.push_back(new Clap(5));
    }
    else{
        logs << ERR << "Color ???";
        exit(EXIT_FAILURE);
    }

    for(unsigned int i = 0 ; i < 8 ; i++)
        listObj.push_back(new Spot(i, capTeam->getColor(), robot->env->obs));

    for(unsigned int i = 0 ; i < 5 ; i++)
        listObj.push_back(new Cup(i, robot->env->obs));

    for(unsigned int i = 0 ; i < 3 ; i++)
        listObj.push_back(new DropCup(i, capTeam->getColor()));

    if(capTeam->getColor() == YELLOW){
        for(unsigned int i = 12 ; i < 20 ; i++){
            robot->env->obs[i].active = 0;
            robot->env->obs_updated[i]++;
        }
    }
    else if(capTeam->getColor() == GREEN){
        for(unsigned int i = 4 ; i < 12 ; i++){
            robot->env->obs[i].active = 0;
            robot->env->obs_updated[i]++;
        }
    }

    if(capTeam->getColor() == GREEN)
        robot->env->obs[BLOCK_START_ZONE].c = {45, 100};
    else
        robot->env->obs[BLOCK_START_ZONE].c = {300-45, 100};

    robot->env->obs[BLOCK_START_ZONE].active = 1;
    robot->env->obs[BLOCK_START_ZONE].r = 20. + R_ROBOT;

    robot->env->obs_updated[BLOCK_START_ZONE]++;

}


