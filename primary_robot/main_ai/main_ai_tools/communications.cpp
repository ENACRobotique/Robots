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
void sendPing(NodesNetwork& nodes){
    int ret;

    logs << INFO << "Pings :";

    for(std::map<nameNode, cfgNode>::iterator it=nodes.nodes.begin() ; it != nodes.nodes.end() ; ++it){
        while((ret = bn_ping(it->second.addr)) < 0)
            logs << ERR << "Can't ping " << it->first << ", error : " << getErrorStr(ret);
        logs << INFO << "Ping " << it->first << " in " << ret << "ms";
    }
}


int roleSetup(bool simu_ai, bool simu_prop, bool simu_beacons){
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


        if(!simu_beacons){
            msg.header.type = E_ROLE_SETUP;
            msg.header.destAddr = ADDRX_MAIN_TURRET;
            msg.payload.roleSetup.nb_steps = 1;
            msg.header.size = 2 + 4*msg.payload.roleSetup.nb_steps;
            // step #0
            msg.payload.roleSetup.steps[0].step_type = UPDATE_ADDRESS;
            msg.payload.roleSetup.steps[0].role = ROLE_PRIM_AI;
            msg.payload.roleSetup.steps[0].address = ADDRD1_MAIN_AI_SIMU;

            bnSendBlock(msg, "FAILED ROLE SETUP 5 (BEACONS)");
        }
    }

    return 0;
}


int syncSetup(bool turet){
    int ret;
    logs << INFO << "Start synchronization";

    if(!turet){ // sync is done by IA, and only between IA and prop
        logs << INFO << "IA" << role_get_addr(ROLE_PRIM_PROPULSION);
        if((ret = bn_intp_sync(role_get_addr(ROLE_PRIM_PROPULSION), 50)) < 0){
            logs << ERR << "FAILED SYNC: " << getErrorStr(-ret) << "(#" << -ret << ")\n";
            return -1;
        }
    }
    else{ // sync is done by turret
        sMsg queryMsg;
        queryMsg.header.destAddr = ADDRI_MAIN_TURRET;
        queryMsg.header.type = E_SYNC_QUERY;
        // beacons
        queryMsg.payload.syncQuery.cfgs[0].type = SYNCTYPE_BEACONS;
        // main AI
        queryMsg.payload.syncQuery.cfgs[1].type = SYNCTYPE_ROLE;
        queryMsg.payload.syncQuery.cfgs[1].role = ROLE_PRIM_AI;
        // prop
        queryMsg.payload.syncQuery.cfgs[2].type = SYNCTYPE_ROLE;
        queryMsg.payload.syncQuery.cfgs[2].role = ROLE_PRIM_PROPULSION;
        // arduino IO
        queryMsg.payload.syncQuery.cfgs[3].type = SYNCTYPE_ADDRESS;
        queryMsg.payload.syncQuery.cfgs[3].addr = ADDRI_MAIN_IO;
        // set sizes
        queryMsg.payload.syncQuery.nb = 4;
        queryMsg.header.size = queryMsg.payload.syncQuery.nb * sizeof(queryMsg.payload.syncQuery.cfgs[0]);
        bnSendBlock(queryMsg, "sync turret");

        eSyncStatus device[queryMsg.payload.syncQuery.nb];
        for(int i = 0; i < queryMsg.payload.syncQuery.nb; i++){
        	device[i] = SYNCSTATUS_TODO;
        }

        sMsg msgIn;

        while(1){
            ret = bn_receive(&msgIn);

            if(ret < 0){
                logs << ERR << "bn_receive() error #" << -ret;
                return -2;
            }

            if( ret > 0){

                switch(msgIn.header.type)
                {
                    case E_SYNC_RESPONSE:
                        cout << endl << "sz: " << (int)msgIn.header.size << "; nb: " << (int)msgIn.payload.syncResponse.nb << "; seq: " << (int)msgIn.header.seqNum << flush;

                        for(int i = 0; i < msgIn.payload.syncResponse.nb; i++){
                            auto& cfg = msgIn.payload.syncResponse.cfgs[i];

                            logs << INFO << "Handled message: " << cfg.type << "; sta: " << cfg.status;
                            switch(cfg.type){
                                case SYNCTYPE_ADDRESS:
                                    logs << "; addr: " << hex << cfg.addr << dec;
                                    break;
                                case SYNCTYPE_ROLE:
                                    logs << "; role: ";
                                    switch(cfg.role){
                                        case ROLE_DEBUG:
                                            logs << "DEBUG";
                                            break;
                                        case ROLE_MONITORING:
                                            logs << "MONITORING";
                                            break;
                                        case ROLE_PRIM_AI:
                                            logs << "PRIM_AI";
                                            break;
                                        case ROLE_PRIM_PROPULSION:
                                            logs << "PRIM_PROPULSION";
                                            break;
                                        case ROLE_SEC_AI:
                                            logs << "SEC_AI";
                                            break;
                                        case ROLE_SEC_PROPULSION:
                                            logs << "SEC_PROPULSION";
                                            break;
                                        default:
                                        case ROLE_UNDEFINED:
                                            logs << "UNDEFINED";
                                            break;
                                    }
                                    break;
                            }

                            if(cfg.status != SYNCSTATUS_OK){
                                logs << ERR << "Sync failed for previous config";
                                return -3;
                            }
                            else{
                                for(int i = 0; i < queryMsg.payload.syncQuery.nb; i++){
                                    if(cfg.type == queryMsg.payload.syncQuery.cfgs[i].type &&
                                            ((cfg.type == SYNCTYPE_ADDRESS && cfg.addr == queryMsg.payload.syncQuery.cfgs[i].addr) ||
                                                    (cfg.type == SYNCTYPE_ROLE && queryMsg.payload.syncQuery.cfgs[i].role) || cfg.type == SYNCTYPE_BEACONS)){
                                        device[i] = SYNCSTATUS_OK;
                                    }
                                }
                            }
                        }
                        break;
                    default:
                        logs << WAR << "Unhandled message of type: " << eType2str((E_TYPE)msgIn.header.type) << " from:" << hex << msgIn.header.srcAddr << dec;
                        break;
                }

                bool ok = true;
                for(int i = 0; i < queryMsg.payload.syncQuery.nb; i++){
                    ok = ok && (device[i] == SYNCSTATUS_OK);
                }
                if(ok)
                    break;

                // TODO print/display/say list of unsynced devices and theyr status to user,
                // TODO perform appropriate actions (retry)
            }
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
int sendSetPosPrimary(
        const Point2D<float> &p, const float theta, // position
        const float p_a_var, const float p_b_var, const float p_a_angle, const float theta_var) { // position uncertainty
    sMsg msgOut ;
    memset(&msgOut, 0, sizeof(msgOut));
   // int ret;

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

    msgOut.payload.genericPosStatus.pos_u.a_var = p_a_var;
    msgOut.payload.genericPosStatus.pos_u.b_var = p_b_var;
    msgOut.payload.genericPosStatus.pos_u.a_angle = p_a_angle;
    msgOut.payload.genericPosStatus.pos_u.theta_var = theta_var;
    msgOut.payload.genericPosStatus.prop_status.action = PROP_SETPOS;

    roleSendBlock(msgOut, ROLEMSG_PRIM_POS, "Send position");
  /*  if ((ret = role_sendRetry(&msgOut, ROLEMSG_PRIM_POS, MAX_RETRIES)) <= 0) {
        logs << ERR << "bn_sendRetry(E_POS) error #" << -ret;
        return -2;
    }*/
    logs << MES << fixed << setprecision(2) << "[POS] Sending position to primary robot (" << msgOut.payload.genericPosStatus.pos.x << ", " << msgOut.payload.genericPosStatus.pos.y << ", " << msgOut.payload.genericPosStatus.pos.theta * 180. / M_PI << ")";

    statuses.posSend(ELT_PRIMARY, p);
    return 1;
}

/*
 * Send a new position imposed to the robot
 */
int sendMixPosPrimary(
        const Point2D<float> &p, const float theta, // position
        const float p_a_var, const float p_b_var, const float p_a_angle, const float theta_var) { // position uncertainty
    static int index = 1;
    sMsg msgOut ;
    memset(&msgOut, 0, sizeof(msgOut));
   // int ret;

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

    msgOut.payload.genericPosStatus.pos_u.a_var = p_a_var;
    msgOut.payload.genericPosStatus.pos_u.b_var = p_b_var;
    msgOut.payload.genericPosStatus.pos_u.a_angle = p_a_angle;
    msgOut.payload.genericPosStatus.pos_u.theta_var = theta_var;
    msgOut.payload.genericPosStatus.prop_status.action = PROP_MIXPOS;
    msgOut.payload.genericPosStatus.prop_status.rid = index;

    roleSendBlock(msgOut, ROLEMSG_PRIM_POS, "Send position");
  /*  if ((ret = role_sendRetry(&msgOut, ROLEMSG_PRIM_POS, MAX_RETRIES)) <= 0) {
        logs << ERR << "bn_sendRetry(E_POS) error #" << -ret;
        return -2;
    }*/
    logs << MES << fixed << setprecision(2) << "[POS] Sending position to primary robot (" << msgOut.payload.genericPosStatus.pos.x << ", " << msgOut.payload.genericPosStatus.pos.y << ", " << msgOut.payload.genericPosStatus.pos.theta * 180. / M_PI << ")";

    statuses.posSend(ELT_PRIMARY, p);
    index++;
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

bool askObsCfg(bool get){
    static bool ask = true; // True if monitoring want receive the configuration of the playground

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
