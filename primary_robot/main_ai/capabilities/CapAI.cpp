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
#include "CapTeam.h"
#include "tools.h"
#include "ai_tools.h"
#include "obj_tools.h"
#include "communications.h"
extern "C"{
#include "millis.h"
}
#include "clap.h"
#include "spot.h"


int CapAI::loop(){
    static estate_t state = COLOR_SELECTION;
    static bool mode_obj = false;
    static int current_obj = -1;
    static Point2D<float> pt_select;
    static unsigned int last_time = 0;
    static unsigned int start_time = 0;

    CapIO* capIO = dynamic_cast<CapIO*> (robot->caps[eCap::IO]);
    CapPropulsion* capProp = dynamic_cast<CapPropulsion*> (robot->caps[eCap::PROP]);
    CapPosition* capPos = dynamic_cast<CapPosition*> (robot->caps[eCap::POS]);
    CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);

    if(!capIO)
        logs << ERR << "IA must be have an HMI interface real or simulate";

    switch (state){
        case COLOR_SELECTION: //Choose the color and take off the starting cord
            capIO->selectColor();

            if(capIO->getHMI(IHM_STARTING_CORD) == CORD_OUT) {
                float theta_robot;

                if (capTeam->getColor() == YELLOW) {
                    logs << INFO << "Color selected is YELLOW";
                    obs[0].c.x = INIT_POS_YELLOW_X;
                    obs[0].c.y = INIT_POS_YELLOW_Y;
                    theta_robot = INIT_ANGLE_YELLOW;
                }
                else if (capTeam->getColor() == GREEN) {
                    logs << INFO << "Color selected is GREEN";
                    obs[0].c.x = INIT_POS_GREEN_X;
                    obs[0].c.y = INIT_POS_GREEN_Y;
                    theta_robot = INIT_ANGLE_GREEN;
                }
                else {
                    logs << ERR << "Error selection color";
                    return -1;
                }


                initObjective();

                Point2D<float> p(obs[0].c.x, obs[0].c.y);
                sendPos(p, theta_robot); //Sending approximate initial position

                //TODO procedure de mise en place

                state = WAIT_STARTING_CORD;
            }
            break; //FIXME WAIT_POS delete because the ia doesn't known if abs pos is active or not, prop give the pos (or status)

        case WAIT_STARTING_CORD: //Wait to take in the starting cord
            if(capIO->getHMI(IHM_STARTING_CORD) == CORD_IN)
                state = WAIT_START;
            break;

        case WAIT_START: //Wait the start (take off the starting cord)
            if (capIO->getHMI(IHM_STARTING_CORD) == CORD_OUT) {
                state = WAIT_SECONDARY;
                last_time = start_time = millis();
            }
            break;

        case WAIT_SECONDARY: //Waiting the secondary unblock the path
            state = GAME;
            break;

        case GAME: //Let's go
            if (millis() - start_time > END_MATCH){
                state = SHUT_DOWN;
                break;
            }

            if (!mode_obj) {
                if(listObj.empty()) //Test if all objective have finished
                    logs << INFO << "Objective list is empty";

                if ((millis() - last_time) > 1000){ //Calculation of the next objective
                      last_time = millis();

                    if ((current_obj = next_obj(start_time, listObj)) != -1) {
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
                        path.sendRobot();
                    }
                }

                if ( (pt_select.distanceTo(capPos->getLastPosXY()) < RESO_POS) && (current_obj != -1) ){ //Test if the robot is on the entry point selected
                    mode_obj = true;
                }
            }else{
                if (metObj(current_obj, listObj) == 0){
                    pt_select.x = -1;
                    pt_select.y = -1;
                    mode_obj = false;
                }
            }
            break;

        case SHUT_DOWN:
            logs << INFO << "SHUT_DOWN : time = " << (unsigned int) (millis() - start_time) / 1000;
            path.stopRobot();
            return 0;
            break;

        default:
            logs << ERR << "Unknown state=" << state;
            break;
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
    else
        logs << ERR << "Color ???";

    for(unsigned int i = 0 ; i < 8 ; i++)
        listObj.push_back(new Spot(i, capTeam->getColor()));
}


