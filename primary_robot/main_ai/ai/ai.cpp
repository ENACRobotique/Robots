/*
 * ai.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */



#include <ai.h>
#include <ai_tools.h>
#include "tools.h"
#include <iostream>

extern "C"{
#include <string.h>
#include <stdlib.h>
#include <millis.h>
}

#include "math_ops.h"
#include <obj_tools.h>
#include <ai/obj_statuses.h>
#include <clap.h>
#include <obj.h>

#include "path.h"

using namespace std;

std::vector<Obj*> listObj;
estate_t state = COLOR_SELECTION;
sWaitPos waiting_pos;
unsigned int last_time2 = -1;
int mode_obj=0;

Path path_;


void colissionDetection(){
    sGenericStatus *stPr = getLastPGStatus(ELT_PRIMARY);
    sPt_t ptPr;
    sGenericStatus *stAPr = getLastPGStatus(ELT_ADV_PRIMARY);
    sPt_t ptAPr;
    sGenericStatus *stASc = getLastPGStatus(ELT_ADV_SECONDARY);
    sPt_t ptASc;
    sNum_t d, dot;
    sVec_t v1, v2;
    int contact = 0;

    if (stPr) {
        ptPr.x = stPr->prop_status.pos.x;
        ptPr.y = stPr->prop_status.pos.y;

        if (stAPr) {
            ptAPr.x = stAPr->prop_status.pos.x;
            ptAPr.y = stAPr->prop_status.pos.y;

            distPt2Pt(&ptPr, &ptAPr, &d);
            v1.x = cos(stPr->prop_status.pos.theta);
            v1.y = sin(stPr->prop_status.pos.theta);
            convPts2Vec(&ptPr, &ptAPr, &v2);
            dotVecs(&v1, &v2, &dot);

            if (d < 50 && dot > 0.6 * d) {
                printf("CONTACT PRIM!!!!!!!!!!!!!!!!!!!!!!!!!\n\n"); // TODO
                contact = 1;
            }
        }

        if (stASc) {
            ptASc.x = stASc->prop_status.pos.x;
            ptASc.y = stASc->prop_status.pos.y;

            distPt2Pt(&ptPr, &ptASc, &d);
            v1.x = cos(stPr->prop_status.pos.theta);
            v1.y = sin(stPr->prop_status.pos.theta);
            convPts2Vec(&ptPr, &ptASc, &v2);
            dotVecs(&v1, &v2, &dot);

            if (d < 40 && dot > 0.6 * d) {
                printf("CONTACT SEC!!!!!!!!!!!!!!!!!!!!!!!!!\n\n"); // TODO
                contact = 1;
            }
        }

        if (contact) {
            sMsg outMsg = { { 0 } };

            outMsg.header.type = E_TRAJ;
            outMsg.header.size = sizeof(outMsg.payload.traj);
            outMsg.payload.traj.p1_x = ptPr.x;
            outMsg.payload.traj.p1_y = ptPr.y;
            outMsg.payload.traj.p2_x = ptPr.x;
            outMsg.payload.traj.p2_y = ptPr.y;
            outMsg.payload.traj.seg_len = 0.;

            outMsg.payload.traj.c_x = ptPr.x;
            outMsg.payload.traj.c_y = ptPr.y;
            outMsg.payload.traj.c_r = 0.;
            outMsg.payload.traj.arc_len = 0.;

            outMsg.payload.traj.sid = 0;
            outMsg.payload.traj.tid = ++last_tid;

            role_sendRetry(&outMsg, MAX_RETRIES);
        }
    }
}


void obj_step(eAIState_t AIState) {
    int obj = -1;


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

#ifndef ABS_POS
                setPos(&obs[0].c, theta_robot); //Sending initial position
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
            if (millis() - _start_time > END_MATCH)
                state = SHUT_DOWN;

            if (AIState == E_AI_PROG) {
            }

            else if (AIState == E_AI_AUTO) {
                //Test si tous objectif sont fini, temporaire pour eviter spam à la fin
                /*           temp=0;
                 for(j=0 ; j<NB_OBJ ; j++){
                 if(listObj[j].active==0) temp++;
                 }
                 if(temp==NB_OBJ) state = SHUT_DOWN;
                 */
                //Calculation of the next objective
                if ((((millis() - last_time) > 1000) && (mode_obj == 0))) {
                    printf("[INFO] obs[0] suivi par next_obj(): x=%f & y=%f\n", obs[0].c.x, obs[0].c.y);
                    last_time = millis();

                    if ((obj = next_obj()) != -1) {
                        current_obj = obj;
                        if (checkCurrentPathLenght(path) == 0 || checkRobotBlock() == 1) {
                            path_.sendRobot(); //FIXME send path
                        }
                    }
                }

                //Update position
                if ((millis() - _start_time) > 2000) {
                    // simuSecondary();
                }

                posPrimary();
                //checkRobot2Obj();
                checkRobotBlock();

                if ((millis() - last_time2) > 1000) {
                    last_time2 = millis();
                    //  updateEntryPointTree();
                    printf("Position actuel : x=%f et y=%f\n", _current_pos.x, _current_pos.y);
                    printf("Select : x=%f et y=%f avec fabsx=%f et fabsy=%f\n", pt_select.x, pt_select.y, fabs(pt_select.x - _current_pos.x), fabs(pt_select.y - _current_pos.y));
                }
            }
            else {
                cerr << "[ERROR] [ai.cpp] Error : Unknown AI state" << endl;
            }

            //If the select point is achieved
            if (((fabs(pt_select.x - _current_pos.x) < RESO_POS && fabs(pt_select.y - _current_pos.y) < RESO_POS)) || mode_obj == 1) { //objectif atteint
                //printf("(listObj[current_obj]).type=%d et curent_obj=%d, mode_obj=%d\n",(listObj[current_obj]).type, current_obj, mode_obj);
                //printf("Select : x=%f et y=%f avec fabsx=%f et fabsy=%f\n", pt_select.x,pt_select.y, fabs(pt_select.x-_current_pos.x),fabs(pt_select.y-_current_pos.y));
                //printf("mode_obj=%d", mode_obj);
                if (metObj(current_obj) == 0){
                    mode_obj = 0;
                }
            }

            break;
        case SHUT_DOWN:
            cout << "[INFO] SHUT_DOWN : time = %d\n" << (int) (millis() - _start_time) / 1000 << endl;
            exit(1);
            //TODO arrêt total
            return;
            break;
        default:
            break;
    }
}

int obj_init(eAIState_t AIState) {

    listObj.push_back(new Clap);
    listObj.push_back(new Clap);

    return 0;
}
