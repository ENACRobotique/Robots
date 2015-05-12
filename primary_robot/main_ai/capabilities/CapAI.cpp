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
#include "spot2.h"
#include "spot3.h"
#include "dropSpot.h"
#include "cup.h"
#include "dropCup.h"
#include "light.h"
#include "objStartingZone.h"
#include "environment.h"


void CapAI::updateWaitObj(paramObj& par){

    for (unsigned int i = 0 ; i < par.obj.size() ; i++){
        if(par.obj[i]->state() == WAIT_MES){
            switch(par.obj[i]->type()){
                case E_OBJ_STARTING_ZONE:
                    {
                        int perform = 0;
                        for(Actuator& j : par.act){
                            if(j.type == ActuatorType::ELEVATOR){
                                if(j.elevator.ball && j.elevator.number == 3){
                                    j.elevator.full = 1;
                                    perform++;
                                }
                                else if(!j.elevator.ball && j.elevator.number == 2){
                                    j.elevator.full = 1;
                                    perform++;
                                }
                            }
                        }
                        if(perform == 2){
                            logs << INFO << "Activation OBJ Starting zone";
                            par.obj[i]->state() = ACTIVE;
                        }
                    }
                    break;
                case E_DROP_CUP:
                    for(Actuator& j : par.act){
                        if(j.type == ActuatorType::CUP){
                            if(j.cupActuator.full){
                                par.obj[i]->state() = ACTIVE;
                                break;
                            }
                        }
                    }
                    break;
                default:
                    logs << ERR << "No action define in WAIT_MES for this objective";
            }

        }

    }
}

int CapAI::loop(){
    static bool mode_obj = false;
    static int current_obj = -1;
    static Point2D<float> pt_select;
    static unsigned int last_time = 0;
    static unsigned int start_time = 0;
    static bool prevDetection = false;
    static int contact = 0;

    CapPropulsion* capProp = dynamic_cast<CapPropulsion*> (robot->caps[eCap::PROP]);
    CapPosition* capPos = dynamic_cast<CapPosition*> (robot->caps[eCap::POS]);
    CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);
    CapActuator* capActuator = dynamic_cast<CapActuator*> (robot->caps[eCap::ACTUATOR]);

    float angleRobot = capPos->getLastTheta();
    Point2D<float> posRobot = capPos->getLastPosXY();
    eColor_t color = capTeam->getColor();

    paramObj par = {posRobot,
            angleRobot,
            color,
            robot->env->obs,
            robot->env->obs_updated,
            listObj,
            capActuator->_act};

    start_time = capTeam->getStartGame();

    if (millis() - start_time > END_MATCH){
        logs << INFO << "SHUT_DOWN : time = " << (unsigned int) (millis() - start_time) / 1000;
        path.stopRobot(capProp->getPropType()==HOLO?true:false);
        return 0;
    }

    if(colissionDetection(robot->el, robot->env->obs)){
        if(!prevDetection){
            path.stopRobot(true);
            prevDetection = true;
            return 1;
        }
        contact ++;


    }
    else{
        prevDetection = false;
        contact = 0;
    }

    if (!mode_obj) {
        if(listObj.empty()) //Test if all objective have finished
            logs << INFO << "Objective list is empty";

        if (((millis() - last_time) > 1000) || (contact == 1)){ //Calculation of the next objective
            last_time = millis();

            if ((current_obj = nextObj(start_time, (int) capPos->getIobs(), capProp->getPropType()==AXLE?true:false, par)) != -1) {
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
        if (metObj(current_obj, par) == 0){
            last_time = 0; //mode obj < 1 second
            pt_select.x = -1;
            pt_select.y = -1;
            mode_obj = false;
            updateWaitObj(par);
        }
    }

    return 1;
}

void CapAI::initObjective(){
    CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);

    logs << INFO << "InitOjective for AI";

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
    listObj.push_back(new ObjStartingZone(capTeam->getColor()));

    for(unsigned int i = 0 ; i < 3 ; i++)
        listObj.push_back(new Spot(i, capTeam->getColor(), robot->env->obs));

    for(unsigned int i = 0 ; i < 2 ; i++)
        listObj.push_back(new Spot2(i, capTeam->getColor()));

    listObj.push_back(new Spot3(robot->env->obs, capTeam->getColor()));
/*
    for(unsigned int i = 0 ; i < 2 ; i++)
        listObj.push_back(new DropSpot(i, capTeam->getColor()));
*/
/*
    listObj.push_back(new Light(capTeam->getColor()));
*/
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


