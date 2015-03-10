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

#include "math_ops.h"
#include <obj_tools.h>
#include <clap.h>
#include <spot.h>
#include <obj.h>
#include <tools.h>
#include "communications.h"

using namespace std;

std::vector<Obj*> listObj;

void colissionDetection(){
    sPt_t ptPr = statuses.getLastPosXY(ELT_PRIMARY);
    float anglePr = statuses.getLastOrient(ELT_PRIMARY);
    sPt_t ptAPr = statuses.getLastPosXY(ELT_ADV_PRIMARY);
    sPt_t ptASc = statuses.getLastPosXY(ELT_ADV_SECONDARY);
    sNum_t d, dot;
    sVec_t v1, v2;
    int contact = 0;

    distPt2Pt(&ptPr, &ptAPr, &d);
    v1.x = cos(anglePr);
    v1.y = sin(anglePr);
    convPts2Vec(&ptPr, &ptAPr, &v2);
    dotVecs(&v1, &v2, &dot);

    if (d < 50 && dot > 0.6 * d) {
        logs << INFO << "CONTACT PRIM!!!!!!!!!!!!!!!!!!!!!!!!!"; // TODO
        contact = 1;
    }

    distPt2Pt(&ptPr, &ptASc, &d);
    v1.x = cos(anglePr);
    v1.y = sin(anglePr);
    convPts2Vec(&ptPr, &ptASc, &v2);
    dotVecs(&v1, &v2, &dot);

    if (d < 40 && dot > 0.6 * d) {
        logs << INFO << "CONTACT SEC!!!!!!!!!!!!!!!!!!!!!!!!!"; // TODO
        contact = 1;
    }

    if (contact) {
        path.stopRobot();
    }

}

void initObjective(){
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
        listObj.push_back(new Spot(i));
}


int stepAI() {
    static estate_t state = COLOR_SELECTION;
    static bool mode_obj = false;
    static int current_obj = -1;
    static sPt_t pt_select;
    static unsigned int last_time = 0;

    switch (state) {
        case COLOR_SELECTION: //Choose the color and take off the starting cord
            startColor();
            if (test_tirette()) {
                float theta_robot;

                if (color == YELLOW) {
                    obs[0].c.x = INIT_POS_YELLOW_X;
                    obs[0].c.y = INIT_POS_YELLOW_Y;
                    theta_robot = INIT_ANGLE_YELLOW;
                }
                else if (color == GREEN) {
                    obs[0].c.x = INIT_POS_GREEN_X;
                    obs[0].c.y = INIT_POS_GREEN_Y;
                    theta_robot = INIT_ANGLE_GREEN;
                }
                else {
                    cerr << "[ERROR] [ai.cpp] Error selection color" << endl;
                }

                initObjective();

                sendPos(obs[0].c, theta_robot); //Sending approximate initial position

                //TODO procedure de mise en place

                state = WAIT_STARTING_CORD;
            }
            break; //FIXME WAIT_POS delete because the ia doesn't known if abs pos is active or not, prop give the pos (or status)

        case WAIT_STARTING_CORD: //Wait to take in the starting cord
#if SIMU
            state = WAIT_START;
#else
            if(!test_tirette()) {
                state = WAIT_START;
            }
#endif
            break;

        case WAIT_START: //Wait the start (take off the starting cord)
            if (test_tirette()) {
                state = WAIT_SECONDARY;
                _start_time = millis();
                last_time = _start_time;
            }
            break;

        case WAIT_SECONDARY: //Waiting the secondary unblock the path
            state = GAME;
            break;

        case GAME: //Let's go
            if (millis() - _start_time > END_MATCH){
                state = SHUT_DOWN;
                break;
            }

            if (!mode_obj) {
                //Test if all objective have finished
                if(listObj.empty())
                    logs << INFO << "Objective list is empty";

                //Calculation of the next objective
                if ((millis() - last_time) > 1000) {
                    last_time = millis();

                    if ((current_obj = next_obj()) != -1) {
                        pt_select = listObj[current_obj]->getDestPoint();
                        logs << INFO << "Selected point is (" << pt_select.x << " ; " << pt_select.y << ")";
#ifdef NON_HOLONOMIC
                  /*      int k = listObj[current_obj]->_EP;
                        sObjPt_t ep = listObj[current_obj]->entryPoint(k);
                        updateEndTraj(ep.angleEP,  &ep.c, ep.radiusEP);
                        printEndTraj();*/
#else
                        path.convPathForHolonomic();
#endif
                        sPath_t path_loc = listObj[current_obj]->getPath();
                        path.addPath2(path_loc);
                        path.sendRobot();
                    }
                }

                //Test if the robot is on the entry point selected
                sPt_t pos_robot = statuses.getLastPosXY(ELT_PRIMARY);
                if ( (fabs(pt_select.x - pos_robot.x) < RESO_POS && fabs(pt_select.y - pos_robot.y) < RESO_POS) && (current_obj != -1) ){
                    mode_obj = true;
                }
            } else{
                if (metObj(current_obj) == 0){
                    pt_select.x = -1;
                    pt_select.y = -1;
                    mode_obj = false;
                }
            }

            // Robots simulation
            if ((millis() - _start_time) > 2000) {
                 simuSecondary();
            }

            break;

        case SHUT_DOWN:
            logs << INFO << "SHUT_DOWN : time = " << (unsigned int) (millis() - _start_time) / 1000;

            path.stopRobot();

            return 0;
            break;

        default:
            logs << ERR << "Unknown state=" << state;
            break;
    }

    return 1;
}

int initAI() {

    //Objectives are initialized in the loop step, because here we don't known the color selected.

    return 0;
}
