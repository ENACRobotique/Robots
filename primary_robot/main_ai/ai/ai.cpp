/*
 * ai.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */



#include <a_star_tools.h>
#include <ai.h>
#include <ai_tools.h>
#include <iostream>

extern "C"{
#include <string.h>
#include <stdlib.h>
#include <millis.h>
}

#include "math_ops.h"
#include <obj_tools.h>
#include <main_ai_tools/statuses.h>
#include <clap.h>
#include <spot.h>
#include <obj.h>

#include <main_ai_tools/path.h>
#include <tools.h>
#include "communications.h"

using namespace std;

std::vector<Obj*> listObj;
estate_t state = COLOR_SELECTION;
sWaitPos waiting_pos;
unsigned int last_time2 = -1;



void colissionDetection(){
    sGenericStatus &stPr = statuses.getLastStatus(ELT_PRIMARY);
    sPt_t ptPr;
    sGenericStatus &stAPr = statuses.getLastStatus(ELT_ADV_PRIMARY);
    sPt_t ptAPr;
    sGenericStatus &stASc = statuses.getLastStatus(ELT_ADV_SECONDARY);
    sPt_t ptASc;
    sNum_t d, dot;
    sVec_t v1, v2;
    int contact = 0;

   // if (stPr) {
        ptPr.x = stPr.prop_status.pos.x;
        ptPr.y = stPr.prop_status.pos.y;

    //    if (stAPr) {
            ptAPr.x = stAPr.prop_status.pos.x;
            ptAPr.y = stAPr.prop_status.pos.y;

            distPt2Pt(&ptPr, &ptAPr, &d);
            v1.x = cos(stPr.prop_status.pos.theta);
            v1.y = sin(stPr.prop_status.pos.theta);
            convPts2Vec(&ptPr, &ptAPr, &v2);
            dotVecs(&v1, &v2, &dot);

            if (d < 50 && dot > 0.6 * d) {
                printf("CONTACT PRIM!!!!!!!!!!!!!!!!!!!!!!!!!\n\n"); // TODO
                contact = 1;
            }
 //       }

   //     if (stASc) {
            ptASc.x = stASc.prop_status.pos.x;
            ptASc.y = stASc.prop_status.pos.y;

            distPt2Pt(&ptPr, &ptASc, &d);
            v1.x = cos(stPr.prop_status.pos.theta);
            v1.y = sin(stPr.prop_status.pos.theta);
            convPts2Vec(&ptPr, &ptASc, &v2);
            dotVecs(&v1, &v2, &dot);

            if (d < 40 && dot > 0.6 * d) {
                printf("CONTACT SEC!!!!!!!!!!!!!!!!!!!!!!!!!\n\n"); // TODO
                contact = 1;
            }
   //     }

        if (contact) {
            path.stopRobot();
        }
   // }
}


void obj_step(eAIState_t AIState) {
    //int obj = -1;
    static bool mode_obj = false;
    static int current_obj = -1;
    sPath_t path_loc;

    switch (state) {
        case COLOR_SELECTION: //Choose the color and take off the starting cord
            startColor();
            if (test_tirette()) {

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


                //init objective
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

#ifndef ABS_POS
                sendPos(obs[0].c, theta_robot); //Sending initial position
                state = WAIT_STARTING_CORD;
#else
                waiting_pos.next = WAIT_STARTING_CORD;
                waiting_pos.pos = obs[0].c;
                waiting_pos.theta = theta_robot;
                state = WAITING_POS;
#endif
            }
            break;

#ifdef ABS_POS
        case WAITING_POS: { //Check if the position of the robot is correct (necessary if we use programmed path for the initialization of the position).
            sNum_t dist;

            distPt2Pt(&waiting_pos.pos, &obs[0].c, &dist);

            if (dist <= 1. /* XXX test theta aswell */) {
                state = waiting_pos.next;
            }
            break;
        }
#endif

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
                if(listObj.empty()){
                    cout << "[INFO] [ai.cpp] Objective list is empty" << endl;
                    state = SHUT_DOWN; //FIXME waitting other objective
                    break;
                }

                //Calculation of the next objective
                if ((millis() - last_time) > 1000) {
                    cout << "[INFO] [ai.cpp] obs[0] suivi par next_obj(): x=" << obs[0].c.x << " & y=" << obs[0].c.y << endl;
                    last_time = millis();

                    if ((current_obj = next_obj()) != -1) {
                        pt_select = listObj[current_obj]->getDestPoint();
                        logs << INFO << "point selected x=" << pt_select.x << " and y=" << pt_select.y;
#ifdef NON_HOLONOMIC
                  /*      int k = listObj[current_obj]->_EP;
                        sObjPt_t ep = listObj[current_obj]->entryPoint(k);
                        updateEndTraj(ep.angleEP,  &ep.c, ep.radiusEP);
                        printEndTraj();*/
#else
                        path.convPathForHolonomic();
#endif
                        path_loc = listObj[current_obj]->getPath();
                        path.addPath2(path_loc);
                        path.sendRobot();
                    }
                }

                //Test is the robot is on the entry point selected
                sPt_t pos_robot = statuses.getLastPosXY(ELT_PRIMARY);
                if ( (fabs(pt_select.x - pos_robot.x) < RESO_POS && fabs(pt_select.y - pos_robot.y) < RESO_POS) && (current_obj != -1) ){
                    mode_obj = true;
                }
            } else{
                cout << mode_obj<< endl;
                if (metObj(current_obj) == 0){
                    pt_select.x = -1;
                    pt_select.y = -1;
                    mode_obj = false;
                }
            }

            //Update position
            if ((millis() - _start_time) > 2000) {
                 simuSecondary();
            }

            obs_updated[4]++;
            posPrimary();
            //checkRobot2Obj();
            //checkRobotBlock();

            if ((millis() - last_time2) > 1000) {
                last_time2 = millis();
           //     printf("Select : x=%f et y=%f avec fabsx=%f et fabsy=%f\n", pt_select.x, pt_select.y, fabs(pt_select.x - _current_pos.x), fabs(pt_select.y - _current_pos.y));
            }

            break;
        case SHUT_DOWN:
            cout << "[INFO] SHUT_DOWN : time = %d\n" << (int) (millis() - _start_time) / 1000 << endl;
            exit(1);
            //TODO arrêt total
            return;
            break;
        default:
            cerr << "[ERROR] [ai.cpp] Unknown state=" << state << endl;
            break;
    }
}

int obj_init(eAIState_t AIState) {
    Path path;

    //path.go2Point(obs[0].c, {100, 100}, true);

    return 0;
}
