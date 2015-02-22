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


#include <astar_tools.h>
#include <tools.h>
#include "ai_types.h"

void sendObsCfg(){
    sMsg msgOut;
    int ret;
/*
    send_obss_reset = 1;
    send_obss_idx = 0;
    prevSendObss = millis();

    for (unsigned int i = 0; i < N; i++) { //TODO change emplacement
        obs_updated[i] = 1; //update all
    }
*/
    msgOut.header.destAddr = role_get_addr(ROLE_MONITORING);
    msgOut.header.type = E_OBS_CFG;
    msgOut.header.size = sizeof(msgOut.payload.obsCfg);

    msgOut.payload.obsCfg.nb_obs = N;
    msgOut.payload.obsCfg.r_robot = R_ROBOT;
    msgOut.payload.obsCfg.x_min = X_MIN;
    msgOut.payload.obsCfg.x_max = X_MAX;
    msgOut.payload.obsCfg.y_min = Y_MIN;
    msgOut.payload.obsCfg.y_max = Y_MAX;

    ret = bn_send(&msgOut);
    if (ret < 0) {
        printf("bn_send(E_OBS_CFG) error #%i\n", -ret);
    }else{
        cout << "[OBS_CFG] message send" << endl;
    }
}

void sendObss(){
    sMsg msgOut;
    static unsigned int prevSendObss;
    static int send_obss_idx = 0;
    int i, ret;

    if (millis() - prevSendObss > 150){
        prevSendObss = millis();

        for (i = 0 ; i < N ; i++){
            if(obs_updated[i] > 0)
                break;
        }
        if( i < N){ //send a message
            msgOut.header.destAddr = role_get_addr(ROLE_MONITORING);
            msgOut.header.type = E_OBSS;

            for (i = 0 ; send_obss_idx < N && i < MAX_NB_OBSS_PER_MSG ; send_obss_idx++) {
                if (obs_updated[send_obss_idx] > 0){
                    obs_updated[send_obss_idx] = 0;

                    msgOut.payload.obss.obs[i].id = send_obss_idx;
                    msgOut.payload.obss.obs[i].active = obs[send_obss_idx].active;
                    msgOut.payload.obss.obs[i].moved = obs[send_obss_idx].moved;

                    msgOut.payload.obss.obs[i].x = (int16_t) (obs[send_obss_idx].c.x * 100. + 0.5);
                    msgOut.payload.obss.obs[i].y = (int16_t) (obs[send_obss_idx].c.y * 100. + 0.5);
                    msgOut.payload.obss.obs[i].r = (int16_t) (obs[send_obss_idx].r * 100. + 0.5);

                    i++;
                }
            }
            msgOut.payload.obss.nb_obs = i;
            msgOut.header.size = 2 + (i << 3);

            ret = bn_send(&msgOut);
            if (ret < 0) {
                printf("bn_send(E_OBSS) error #%i\n", -ret);
            }else {
                cout << "[OBSS] message send" << endl;
            }

            if (send_obss_idx == N) {
                send_obss_idx = 0;
            }
        }
    }
}
/*
 * Send the position of the robot
 * Failure if return is < 0
 */
void sendPos(sPt_t &p, sNum_t theta) {
    int ret;

    sMsg msg = { { 0 } };
    msg.header.type = E_POS;
    msg.header.size = sizeof(msg.payload.pos);

    msg.payload.pos.id = 0;
    msg.payload.pos.u_a = 0;
    msg.payload.pos.u_a_theta = 0;
    msg.payload.pos.u_b = 0;
    msg.payload.pos.theta = theta;
    msg.payload.pos.x = p.x;
    msg.payload.pos.y = p.y;
    obs[0].c.x = p.x;
    obs[0].c.y = p.y;
    theta_robot = theta;
    _current_pos = obs[0].c;

    if ((ret = role_sendRetry(&msg, MAX_RETRIES)) <= 0) {
        printf("bn_sendRetry(E_POS) error #%i\n", -ret);
    }
    else {
        printf("Sending position to robot%i (%.2fcm,%.2fcm,%.2f°).\n", msg.payload.pos.id, msg.payload.pos.x, msg.payload.pos.y, msg.payload.pos.theta * 180. / M_PI);
    }
}


int ping(){
#if !SIMU
    int state = 0, ret ;
    while(1){
        switch(state){
            //Minimum necessary
            case 0:
                if( (ret = bn_ping(ADDRD1_DBGBRIDGE)) >= 0){ //FIXME bn_ping doesn't exist
                    state = 1;
                }
                printf("Ping debug bridge : %d\n", ret);
                break;
            case 1:
                if( (ret = bn_ping(ADDRU1_MAIN_IO)) >= 0){
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
           //Optional (for the moment)
            case 3:
                if( (ret = bn_ping(ADDRD1_MONITORING)) < 0){
                    printf("Warning : Monitoring is not connected - ");
                    }
                state = 4;
                printf("Ping monitoring : %d\n", ret);
                break;
            case 4:
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
                state = 6;
                printf("Ping mobile 2 : %d\n", ret);
                break;

            }
        if(state == 6) break;
        }
#endif
    return 1;
}


void checkInbox(int verbose, ofstream &file){
    sMsg msgIn;
    int ret;

    printf("\x1b[K\x1b[s");

    ret = bn_receive(&msgIn);
    if (ret > 0) {
        // get the message
        ret = role_relay(&msgIn);
        if ( ret < 0 ) {
            cerr << "role_relay() error #" << -ret << endl;
        }

        // print message
        if (verbose >= 1) {
            if (msgIn.header.type != E_POS)
                cout << "[NEW MES] message received from : " << role_string(role_get_role(msgIn.header.srcAddr)) << "(" << msgIn.header.srcAddr << "), type : " << eType2str((E_TYPE) msgIn.header.type) << "(" << msgIn.header.type << ")" << endl;
            else if (verbose >= 2) {
                cout << "[NEW MES] message received from : " << role_string(role_get_role(msgIn.header.srcAddr)) << "(" << msgIn.header.srcAddr << "), type : " << eType2str((E_TYPE) msgIn.header.type) << "(" << msgIn.header.type << ")" << endl;
            }

            if (file)
                file << "[NEW MES] message received from : " << role_string(role_get_role(msgIn.header.srcAddr)) << "(" << msgIn.header.srcAddr << "), type : " << eType2str((E_TYPE) msgIn.header.type) << "(" << msgIn.header.type << ")" << endl;
        }

        // processing of the message
        switch (msgIn.header.type) {
            case E_DEBUG:
                cout << "[DEBUG]" << msgIn.payload.debug << endl;
                if (file)
                    file << "[DEBUG]" << msgIn.payload.debug << endl;
                break;
            case E_POS:
                if( verbose >= 2)
                   printf("[POS] robot%hhu@(%fcm,%fcm,%f°)\n", msgIn.payload.pos.id, msgIn.payload.pos.x, msgIn.payload.pos.y, msgIn.payload.pos.theta*180./M_PI);
                if (file){}
                   //TODO fprintf(fd, "message received from %hx, type : %s (%hhu)  ", msgIn.header.srcAddr, eType2str((E_TYPE) msgIn.header.type), msgIn.header.type);
                   //TODO created a real file log
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

           /*     if (curr_path.path && msgIn.payload.pos.tid == curr_path.tid) {
                    if (msgIn.payload.pos.ssid) { // circle portion
                        sPt_t *o_c = NULL;
                        sVec_t v;
                        sNum_t r, n, d;

                        o_c = &curr_path.path[msgIn.payload.pos.sid].obs.c;
                        r = curr_path.path[msgIn.payload.pos.sid].obs.r;

                        convPts2Vec(o_c, &last_pos, &v);
                        normVec(&v, &n);

                        d = n - fabs(r);

                        if (fabs(d) > 2.) {
                            printf("!!! far from the circle (%.2fcm)...\n", d);
                        }

                        obs[0].moved = 1;
                        obs[0].c.x = o_c->x + v.x * (fabs(r) + 0.1) / n;
                        obs[0].c.y = o_c->y + v.y * (fabs(r) + 0.1) / n;
                        obs[0].r = 0.;
                    }
                    else { // line portion
                        sNum_t d;
                        sPt_t h;
                        sSeg_t s;

                        convPts2Seg(&curr_path.path[msgIn.payload.pos.sid].p1, &curr_path.path[msgIn.payload.pos.sid].p2, &s);

                        sqdistPt2Seg(&last_pos, &s, &d, &h);

                        if (d > 2. * 2.) {
                            printf("!!! far from the line (%.2fcm)...\n", sqrt(d));
                        }

                        obs[0].moved = 1;
                        obs[0].c = h;
                        obs[0].r = 0.;
                    }
                }
                else {
                    obs[0].moved = 1;
                    obs[0].c = last_pos;
                    obs[0].r = 0.;
                }*/
                break;
            case E_GOAL: //FIXME No used
                printf("[GOAL] robot%hhu@(%.2fcm,%.2fcm,%.2f°)\n", msgIn.payload.pos.id, msgIn.payload.pos.x, msgIn.payload.pos.y, msgIn.payload.pos.theta * 180. / M_PI);
               // if (file)
               //     fprintf(fd, "message received from %hx, type : %s (%hhu)  ", msgIn.header.srcAddr, eType2str((E_TYPE) msgIn.header.type), msgIn.header.type);

                sPt_t goal;
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
                    cout << "[GENERIC_STATUS]" << endl;

                statuses.receivedNewStatus(msgIn.payload.genericStatus);
                printf("pos:%.2f,%.2f\n", msgIn.payload.genericStatus.adv_status.pos.x, msgIn.payload.genericStatus.adv_status.pos.y);
                break;
            case E_IHM_STATUS:
                /*
                 for(i = 0 ; i < (int)msgIn.payload.ihmStatus.nb_states ; i++){
                 switch(msgIn.payload.ihmStatus.states[i].id){
                 case IHM_STARTING_CORD:
                 starting_cord = msgIn.payload.ihmStatus.states[i].state;
                 printf("## scord: %i\n", starting_cord);
                 break;
                 case IHM_MODE_SWICTH:
                 mode_switch = msgIn.payload.ihmStatus.states[i].state;
                 printf("## smode: %i\n", mode_switch);
                 break;
                 case IHM_LED:
                 break;
                 case IHM_LIMIT_SWITCH_LEFT:
                 switch_left = msgIn.payload.ihmStatus.states[i].state;
                 printf("## lswitch: %i\n", switch_left);
                 break;
                 case IHM_LIMIT_SWITCH_RIGHT:
                 switch_right = msgIn.payload.ihmStatus.states[i].state;
                 printf("## rswitch: %i\n", switch_right);
                 break;
                 default:
                 break;
                 }
                 }
                 */
                break;
            default:
                printf("\n");
            /*    if (fd)
                    fprintf(fd, "\n");*/
                break;
        }

        //TODO used statuses : printf("pos %.2fcm, %.2fcm, %.1f°", statuses., last_pos.y, last_theta * 180. / M_PI);
        printf("\x1b[u");
       //C99 standard FIXME fflush(stdout);
    }else if(ret < 0){
        fprintf(stderr, "bn_receive() error #%i\n", -ret);
        exit(1);
    }

}

/*
 * Return true if a new goal is available.
 * get == true -> get
 * get == false -> set
 */

bool lastGoal(sPt_t &goal, bool get){
    static sPt_t pt;
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
