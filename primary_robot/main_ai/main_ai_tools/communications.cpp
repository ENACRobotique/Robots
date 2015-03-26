/*
 * communications_tools.cpp
 *
 *  Created on: 22 févr. 2015
 *      Author: seb
 */

#include <communications.h>
#include <iostream>
#include <cmath>

extern "C"{
#include <stdio.h>

#include "millis.h"
#include "messages.h"
#include "roles.h"
}

#include "botNet_core.h"
#include "bn_utils.h"


#include <a_star_tools.h>
#include <tools.h>
#include "ai_types.h"


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
        }*/
}

/*
 * Send the limit of the playground to monitoring
 */
void sendObsCfg(){
    sMsg msgOut;
    int ret;

    msgOut.header.destAddr = role_get_addr(ROLE_MONITORING);
    msgOut.header.type = E_OBS_CFG;
    msgOut.header.size = sizeof(msgOut.payload.obsCfg);

    msgOut.payload.obsCfg.nb_obs = N;
    msgOut.payload.obsCfg.r_robot = R_ROBOT;
    msgOut.payload.obsCfg.x_min = X_MIN;
    msgOut.payload.obsCfg.x_max = X_MAX;
    msgOut.payload.obsCfg.y_min = Y_MIN;
    msgOut.payload.obsCfg.y_max = Y_MAX;

    if ( (ret = bn_send(&msgOut)) < 0) {
        cerr << "[ERROR] [communications.cpp] bn_send(E_OBS_CFG) error #%i" << -ret << endl;
        return;
    }

    logs << MES << "[OBS_CFG] message send to monitoring";
}

/*
 * Send obs where obs_updated was modified to monitoring
 */
void sendObss(){
    sMsg msgOut;
    static unsigned int prevSendObss;
    static int send_obss_idx = 0;
    int i, ret;

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
                cerr << "[ERROR] [communication.cpp] bn_send(E_OBSS) error #%i" << -ret << endl;
                return;
            }

            logs << MES <<"[OBSS] message send to monitoring";

        }
    }
}

/*
 * Send a new position imposed to the robot
 */
int sendPos(sPt_t &p, sNum_t theta) {
    sMsg msgOut ;
    int ret;

    if ((p.x < 0.) || (p.x > 300.) || (p.y < 0.) || (p.y > 200.))
        return -1;

    msgOut.header.type = E_POS;
    msgOut.header.size = sizeof(msgOut.payload.pos);

    msgOut.payload.pos.id = 0;
    msgOut.payload.pos.u_a = 0;
    msgOut.payload.pos.u_a_theta = 0;
    msgOut.payload.pos.u_b = 0;
    msgOut.payload.pos.theta = theta;
    msgOut.payload.pos.x = p.x;
    msgOut.payload.pos.y = p.y;

    //XXX Created an intern message generic status for update the position, and if message not pos not receive --> position problem !!!

    if ((ret = role_sendRetry(&msgOut, MAX_RETRIES)) <= 0) {
        logs << ERR << "bn_sendRetry(E_POS) error #" << -ret;
        return -2;
    }
    logs << MES << "[POS] Sending position to primary robot (" << msgOut.payload.pos.x << ", " << msgOut.payload.pos.y << ", " << msgOut.payload.pos.theta * 180. / M_PI << ")";

    return 1;
}

/*
 * Send a new speed imposed to the robot
 */
int sendSpeed(sNum_t speed) {
    sMsg msgOut;
    int ret;

    if (fabs(speed) > MAX_SPEED)
        return -1;

    msgOut.header.destAddr = role_get_addr(ROLE_PROPULSION);
    msgOut.header.type = E_SPEED_SETPOINT;
    msgOut.header.size = sizeof(msgOut.payload.speedSetPoint);

    msgOut.payload.speedSetPoint.speed = speed;

    if ((ret = bn_sendRetry(&msgOut, MAX_RETRIES)) <= 0) {
        cerr << "[ERROR] [communication.cpp] bn_sendRetry(E_SPEED_SETPOINT) error #" << -ret << endl;
        return -2;
    }
    cout << "[SEND MES] [SPEED_SETPOINT] Sending speed to primary robot (" << msgOut.payload.speedSetPoint.speed << ")" << endl;

    return 1;
}

/*
 * Check if a new message is available and do the appropriate operation
 */
void checkInbox(int verbose){
    sMsg msgIn;
    Point2D<float> goal;
    int ret;

    if((ret = bn_receive(&msgIn)) < 0){ //get the message
        cerr << "[ERROR] [communication.cpp] bn_receive() error #" << -ret << endl;
        return;
    }else if( ret == 0) //no message
        return;

    if ((ret = role_relay(&msgIn)) < 0 ) { // relay the message
        cerr << "[ERROR] [communication.cpp] role_relay() error #" << -ret << endl;
        return;
    }

    // print message
    logs << MES_V(E_V3) << "message received from : " << role_string(role_get_role(msgIn.header.srcAddr)) << "(" << msgIn.header.srcAddr << "), type : " << eType2str((E_TYPE) msgIn.header.type) << "(" << msgIn.header.type << ")";

    // processing of the message
    switch (msgIn.header.type) {
        case E_DEBUG:
            cout << "[DEBUG]" << msgIn.payload.debug << endl;
            break;
        case E_POS:
            logs << MES_V(E_V3) << "[POS] robot" << msgIn.payload.pos.id << "@(" << msgIn.payload.pos.x << ", " << msgIn.payload.pos.y << ", " << msgIn.payload.pos.theta * 180. / M_PI << ")";

            //convert in generic status because E_POS message is depreciate
            sGenericStatus sta;

            sta.date = micros(); // XXX
            sta.id = ELT_PRIMARY;
            sta.prop_status.pos.frame = FRAME_PLAYGROUND;
            sta.prop_status.pos.theta = msgIn.payload.pos.theta;
            sta.prop_status.pos_u.a_var = 0.;
            sta.prop_status.pos_u.b_var = 0.;
            sta.prop_status.pos_u.a_angle = 0.;
            sta.prop_status.pos_u.theta = 0.;
            sta.prop_status.pos.x = msgIn.payload.pos.x;
            sta.prop_status.pos.y = msgIn.payload.pos.y;

            statuses.receivedNewStatus(sta);
            break;
        case E_GOAL:
            cout << "[GOAL] robot" << msgIn.payload.pos.id << "@(" << msgIn.payload.pos.x << ", " << msgIn.payload.pos.y << ", " << msgIn.payload.pos.theta * 180. / M_PI << ")" << endl;

            goal.x = msgIn.payload.pos.x;
            goal.y = msgIn.payload.pos.y;
            lastGoal(goal, false);
            break;
        case E_OBS_CFG:
            cout << "[OBS_CFS]" << endl;

            sendObsCfg();
            break;
        case E_GENERIC_STATUS:
            if( verbose >= 2)
                cout << "[GENERIC_STATUS] pos:(" << msgIn.payload.genericStatus.adv_status.pos.x << ", " << msgIn.payload.genericStatus.adv_status.pos.y << ", " << msgIn.payload.genericStatus.adv_status.pos.theta * 180. / M_PI << ")" << endl;

            statuses.receivedNewStatus(msgIn.payload.genericStatus);
            break;
        case E_IHM_STATUS:
            cout << "[IHM_STATUS] ";

            ihm.receivedNewIhm(msgIn.payload.ihmStatus);
            break;
        default:
            cout << "[WARNING] message type not define or doesn't exist" << eType2str((E_TYPE)msgIn.header.type) << endl;
    }
}

/*
 * Return true if a new goal is available.
 * get == true -> get
 * get == false -> set
 */
bool lastGoal(Point2D<float> goal, bool get){
    static Point2D<float> pt;
    static bool new_goal = false;

    if(!get){
        cout << "[INFO] Save new goal" << endl;
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
