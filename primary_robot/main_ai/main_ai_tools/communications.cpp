/*
 * communications_tools.cpp
 *
 *  Created on: 22 févr. 2015
 *      Author: seb
 */

#include <communications.h>
#include <iostream>
#include <cmath>
#include <cstring>

extern "C"{
#include <stdio.h>
#include "global_errors.h"
#include "millis.h"
#include "messages.h"
#include "roles.h"
}

#include "botNet_core.h"
#include "bn_utils.h"
#include "bn_intp.h"


#include <a_star_tools.h>
#include <tools.h>
#include "ai_tools.h"


void bnSendBlock(sMsg& msg, const string& txt){
    int ret = -1;
    int i = 0;

    do{
        ret = bn_sendAck(&msg);
        if(ret <= 0){
            if(!i){
                logs << ERR << txt << "bn_send failed" << getErrorStr(-ret) << "(#" << -ret << ")";
            }
            i++;
        }
        else if (i){
            logs << WAR << "bn_send success after " << i << "tries";
        }
    }while(ret <= 0);
}

void roleSendBlock(sMsg& msg, eRoleMsgClass mc, const string& txt){
    int ret = -1;
    int i = 0;

    do{
        ret = role_sendAck(&msg, mc);
        if(ret <= 0){
            if(!i){
                logs << ERR << txt << "role_send failed" << getErrorStr(-ret) << "(#" << -ret << ")";
            }
            i++;
        }
        else if (i){
            logs << WAR << "role_send success after " << i << "tries";
        }
    }while(ret <= 0);
}

/*
 * Ping all the interface.
 * If the interface is optional a message warning is print.
 * If the interface is critical a new ping is send in loop and a message error is print.
 */
void sendPing(){
/*
    int state = 0, ret ;
    while(1){
        switch(state){
            //Minimum necessary
            case 0:
#if !SIMU
                if( (ret = bn_ping(ADDRD1_DBGBRIDGE)) >= 0){
                    state = 1;
                }
                printf("Ping debug bridge : %d\n", ret);
                break;
            case 1:
                if( (ret = bn_ping(ADDRI1_MAIN_IO)) >= 0){
                    state = 2;
                    }
                printf("Ping main io : %d\n", ret);
                break;
            case 2:
                if( (ret = bn_ping(ADDRU2_MAIN_PROP)) >= 0){
                    state = 3;
                    }
                printf("Ping main prop : %d\n", ret);
                break;
#else
                if( (ret = bn_ping(ADDRD1_MAIN_PROP_SIMU)) >= 0){
                    state = 3;
                    logs << INFO << "Ping main prop simu : " << ret;
                    break;
                    }
                logs << ERR << "Ping main prop simu error#" << -ret;
                break;
#endif
           //Optional
            case 3:
                if( (ret = bn_ping(ADDRD1_MONITORING)) < 0){
                    logs << WAR << "Monitoring is not connected";
                    }
                state = 4;
                logs << INFO << "Ping monitoring : " << -ret;
                break;
            case 4:
#if !SIMU
                if( (ret = bn_ping(ADDRX_MOBILE_1)) < 0){
                    printf("Warning : Mobile 1 is not connected - ");
                    }
                state = 5;
                printf("Ping mobile 1 : %d\n", ret);
                break;
            case 5:
                if( (ret = bn_ping(ADDRX_MOBILE_2)) < 0){
                    printf("Warning : Mobile 2 is not connected - ");
                    }
                printf("Ping mobile 2 : %d\n", ret);
#endif
                state = 6;
                break;


            }
        if(state == 6) break;
        }
        */
}


int roleSetup(bool simu_ai, bool simu_prop){
    sMsg msg;
    memset(&msg, 0, sizeof(msg));

    if(simu_prop){
        role_set_addr(ROLE_PRIM_PROPULSION, ADDRD1_MAIN_PROP_SIMU);

        msg.header.type = E_ROLE_SETUP;
        msg.header.destAddr = role_get_addr(ROLE_MONITORING);
        msg.payload.roleSetup.nb_steps = 1;
        msg.header.size = 2 + 4*msg.payload.roleSetup.nb_steps;
        // step #0
        msg.payload.roleSetup.steps[0].step_type = UPDATE_ADDRESS;
        msg.payload.roleSetup.steps[0].role = ROLE_PRIM_PROPULSION;
        msg.payload.roleSetup.steps[0].address = ADDRD1_MAIN_PROP_SIMU;

        bnSendBlock(msg, "FAILED ROLE SETUP 1 (MONITORING)");
    }

    if(simu_ai){
        msg.header.type = E_ROLE_SETUP;
        msg.header.destAddr = role_get_addr(ROLE_PRIM_PROPULSION);
        msg.payload.roleSetup.nb_steps = 1;
        msg.header.size = 2 + 4*msg.payload.roleSetup.nb_steps;
        // step #0
        msg.payload.roleSetup.steps[0].step_type = UPDATE_ADDRESS;
        msg.payload.roleSetup.steps[0].role = ROLE_PRIM_AI;
        msg.payload.roleSetup.steps[0].address = ADDRD1_MAIN_AI_SIMU;

        bnSendBlock(msg, "FAILED ROLE SETUP 2 (PRIM_PROPULSION)");


        msg.header.type = E_ROLE_SETUP;
        msg.header.destAddr = role_get_addr(ROLE_MONITORING);
        msg.payload.roleSetup.nb_steps = 1;
        msg.header.size = 2 + 4*msg.payload.roleSetup.nb_steps;
        // step #0
        msg.payload.roleSetup.steps[0].step_type = UPDATE_ADDRESS;
        msg.payload.roleSetup.steps[0].role = ROLE_PRIM_AI;
        msg.payload.roleSetup.steps[0].address = ADDRD1_MAIN_AI_SIMU;

        bnSendBlock(msg, "FAILED ROLE SETUP 3 (MONITORING)");


        msg.header.type = E_ROLE_SETUP;
        msg.header.destAddr = ADDRI_MAIN_IO;
        msg.payload.roleSetup.nb_steps = 1;
        msg.header.size = 2 + 4*msg.payload.roleSetup.nb_steps;
        // step #0
        msg.payload.roleSetup.steps[0].step_type = UPDATE_ADDRESS;
        msg.payload.roleSetup.steps[0].role = ROLE_PRIM_AI;
        msg.payload.roleSetup.steps[0].address = ADDRD1_MAIN_AI_SIMU;

        if(simu_prop){
            if(bn_send(&msg) < 0)
                logs << WAR << "FAILED ROLE SETUP 4 (MAIN_IO)";
        }
        else{
            bnSendBlock(msg, "FAILED ROLE SETUP 4 (MAIN_IO)");
        }
    }

    return 0;
}

/*
 * Send the limit of the playground to monitoring
 */
void sendObsCfg(const int n, const int rRobot, const int xMin, const int xMax, const int yMin, const int yMax){
    sMsg msgOut;
    int ret;
    memset(&msgOut, 0, sizeof(msgOut));

    msgOut.header.destAddr = role_get_addr(ROLE_MONITORING);
    msgOut.header.type = E_OBS_CFG;
    msgOut.header.size = sizeof(msgOut.payload.obsCfg);

    msgOut.payload.obsCfg.nb_obs = n;
    msgOut.payload.obsCfg.r_robot = rRobot;
    msgOut.payload.obsCfg.x_min = xMin;
    msgOut.payload.obsCfg.x_max = xMax;
    msgOut.payload.obsCfg.y_min = yMin;
    msgOut.payload.obsCfg.y_max = yMax;

    if ( (ret = bn_send(&msgOut)) < 0) {
        logs << ERR << "bn_send(E_OBS_CFG) error #%i" << -ret;
        return;
    }

    logs << MES << "[OBS_CFG] message send to monitoring";
}

/*
 * Send obs where obs_updated was modified to monitoring
 */
void sendObss(vector<astar::sObs_t>& obs, vector<uint8_t>& obs_updated){
    sMsg msgOut;
    static unsigned int prevSendObss;
    static int send_obss_idx = 0;
    int i, ret, N = obs.size();
    memset(&msgOut, 0, sizeof(msgOut));

    if (millis() - prevSendObss > 150){
        prevSendObss = millis();

        for (i = 0 ; i < N-1 ; i++){ //check if at least one obs_updated was modified
            if(obs_updated[i] > 0)
                break;
        }
        if( i < N-1 ){ //send a message
            msgOut.header.destAddr = role_get_addr(ROLE_MONITORING);
            msgOut.header.type = E_OBSS;

            for (i = 0 ; send_obss_idx < N-1 && i < MAX_NB_OBSS_PER_MSG ; send_obss_idx++) {
                if (obs_updated[send_obss_idx] > 0){
                    obs_updated[send_obss_idx] = 0;

                    msgOut.payload.obss.obs[i].id = send_obss_idx;
                    msgOut.payload.obss.obs[i].active = obs[send_obss_idx].active;
                    msgOut.payload.obss.obs[i].moved = obs[send_obss_idx].moved;

                    msgOut.payload.obss.obs[i].x = (int16_t) (obs[send_obss_idx].c.x * 100. + 0.5);
                    msgOut.payload.obss.obs[i].y = (int16_t) (obs[send_obss_idx].c.y * 100. + 0.5);
                    msgOut.payload.obss.obs[i].r = (int16_t) (obs[send_obss_idx].r * 100. + 0.5);
                    logs << INFO << "element mis a jour :" << send_obss_idx;
                    i++;
                }
            }
            msgOut.payload.obss.nb_obs = i;
            msgOut.header.size = 2 + (i << 3);

            if (send_obss_idx == N-1)
                send_obss_idx = 0;

            if ((ret = bn_send(&msgOut)) < 0) {
                logs << ERR << "bn_send(E_OBSS) error #%i" << -ret;
                return;
            }

            logs << MES <<"[OBSS] message send to monitoring";

        }
    }
}

/*
 * Send a new position imposed to the robot
 */
int sendPosPrimary(Point2D<float> &p, float theta) {
    sMsg msgOut ;
    memset(&msgOut, 0, sizeof(msgOut));

    if ((p.x < 0.) || (p.x > 300.) || (p.y < 0.) || (p.y > 200.))
        return -1;

    msgOut.header.type = E_GENERIC_POS_STATUS;
    msgOut.header.size = sizeof(msgOut.payload.genericPosStatus);

    msgOut.payload.genericPosStatus.id = ELT_PRIMARY;
    msgOut.payload.genericPosStatus.date = bn_intp_micros2s(micros()); // now

    msgOut.payload.genericPosStatus.pos.frame = FRAME_PLAYGROUND;
    msgOut.payload.genericPosStatus.pos.x = p.x;
    msgOut.payload.genericPosStatus.pos.y = p.y;
    msgOut.payload.genericPosStatus.pos.theta = theta;
    msgOut.payload.genericPosStatus.pos_u.a_var = 10*10;
    msgOut.payload.genericPosStatus.pos_u.b_var = 10*10;
    msgOut.payload.genericPosStatus.pos_u.a_angle = 0;
    msgOut.payload.genericPosStatus.pos_u.theta_var = M_PI*M_PI/16;
    msgOut.payload.genericPosStatus.prop_status.action = PROP_SETPOS;

    roleSendBlock(msgOut, ROLEMSG_PRIM_POS, __func__);

    logs << MES << "[POS] Sending position to primary robot (" << msgOut.payload.genericPosStatus.pos.x << ", " << msgOut.payload.genericPosStatus.pos.y << ", " << msgOut.payload.genericPosStatus.pos.theta * 180. / M_PI << ")";

    statuses.posSend(ELT_PRIMARY, p);
    return 1;
}

/*
 * Send a new speed imposed to the robot
 */
int sendSpeedPrimary(float speed) {
    sMsg msgOut;
    memset(&msgOut, 0, sizeof(msgOut));
    int ret;

    if (fabs(speed) > MAX_SPEED)
        return -1;

    msgOut.header.destAddr = role_get_addr(ROLE_PRIM_PROPULSION);
    msgOut.header.type = E_SPEED_SETPOINT;
    msgOut.header.size = sizeof(msgOut.payload.speedSetPoint);

    msgOut.payload.speedSetPoint.speed = speed;

    if ((ret = bn_sendRetry(&msgOut, MAX_RETRIES)) <= 0) {
        logs << ERR << "bn_sendRetry(E_SPEED_SETPOINT) error #" << -ret;
        return -2;
    }
    logs << MES << "Sending speed to primary robot (" << msgOut.payload.speedSetPoint.speed << ")";

    return 1;
}

/*
 * Check if a new message is available and do the appropriate operation
 */
void checkInbox(int /*verbose*/){
    sMsg msgIn;
    Point2D<float> goal;
    int ret;

    if((ret = bn_receive(&msgIn)) < 0){ //get the message
        logs << ERR << "bn_receive() error #" << -ret;
        return;
    }else if( ret == 0) //no message
        return;

    if ((ret = role_relay(&msgIn)) < 0 ) { // relay the message
        logs << ERR << "role_relay() error #" << -ret;
    }

    // print message
    logs << MES_V(E_V3) << "message received from : " << role_string(role_get_role(msgIn.header.srcAddr)) << "(" << msgIn.header.srcAddr << "), type : " << eType2str((E_TYPE) msgIn.header.type);

    // processing of the message
    switch (msgIn.header.type) {
        case E_DEBUG:
            logs << MES << "[DEBUG]" << msgIn.payload.debug;
            break;
        case E_GENERIC_POS_STATUS:
            logs << MES_V(E_V3) << "[GENERIC_POS_STATUS] robot" << msgIn.payload.genericPosStatus.id << "@(" << msgIn.payload.genericPosStatus.pos.x << ", " << msgIn.payload.genericPosStatus.pos.y << ", " << msgIn.payload.genericPosStatus.pos.theta * 180. / M_PI << ")";

            statuses.receivedNewStatus(msgIn.payload.genericPosStatus);
            break;
        case E_GOAL:
            logs << MES << "[GOAL] robot" << msgIn.payload.genericPosStatus.id << "@(" << msgIn.payload.genericPosStatus.pos.x << ", " << msgIn.payload.genericPosStatus.pos.y << ", " << msgIn.payload.genericPosStatus.pos.theta * 180. / M_PI << ")";

            goal.x = msgIn.payload.genericPosStatus.pos.x;
            goal.y = msgIn.payload.genericPosStatus.pos.y;
            lastGoal(goal, false);
            break;
        case E_OBS_CFG:
            logs << MES << "[OBS_CFS] receive";

            askObsCfg(false);
            break;
        case E_IHM_STATUS:
            ihm.receivedNewIhm(msgIn.payload.ihmStatus);
            break;
        default:
            logs << WAR << "message type not define or doesn't exist : " << eType2str((E_TYPE)msgIn.header.type);
    }
}

/*
 * Return true if a new goal is available.
 * get == true -> get
 * get == false -> set
 */
bool lastGoal(Point2D<float>& goal, bool get){
    static Point2D<float> pt;
    static bool new_goal = false;

    if(!get){
        logs << INFO << "Save new goal";
        pt = goal;
        new_goal = true;
        return true;
    }else if(new_goal){
        goal = pt;
        new_goal = false;
        return true;
    }else
        return false;
}

bool askObsCfg(bool get){
    static bool ask = true; // True if monitoring want receive the congiguration of the playground

    if(!get){
        ask = true;
        return true;
    }
    else if(ask){
        ask = false;
        return true;
    }
    else
        return false;
}
