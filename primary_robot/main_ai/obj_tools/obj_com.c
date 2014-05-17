/*
 * obj_com.c
 *
 *  Created on: 18 avr. 2014
 *      Author: seb
 */


#include "obj_com.h"

#include <math.h>
#include "../botNet/shared/botNet_core.h"


void send_robot(sPath_t path){
    sMsg outMsg = {{0}};
    int i, ret ;
    static int tid = 0;
    tid++;
    if(path.path)
        for(i = 0; i < path.path_len; i++) {
            printf("  %u: p1 x%f y%f, p2 x%f y%f, obs x%f y%f r%.2f, a_l%f s_l%f\n", i, path.path[i].p1.x, path.path[i].p1.y, path.path[i].p2.x, path.path[i].p2.y,path.path[i].obs.c.x,path.path[i].obs.c.y, path.path[i].obs.r,path.path[i].arc_len,path.path[i].seg_len);

            outMsg.header.type = E_TRAJ;
            outMsg.header.size = sizeof(outMsg.payload.traj);

            outMsg.payload.traj.p1_x = path.path[i].p1.x;
            outMsg.payload.traj.p1_y = path.path[i].p1.y;
            outMsg.payload.traj.p2_x = path.path[i].p2.x;
            outMsg.payload.traj.p2_y = path.path[i].p2.y;
            outMsg.payload.traj.seg_len = path.path[i].seg_len;

            outMsg.payload.traj.c_x = path.path[i].obs.c.x;
            outMsg.payload.traj.c_y = path.path[i].obs.c.y;
            outMsg.payload.traj.c_r = path.path[i].obs.r;
            outMsg.payload.traj.arc_len = path.path[i].arc_len;

            outMsg.payload.traj.sid = i;
            outMsg.payload.traj.tid = tid;

            ret = role_send(&outMsg);
            if(ret < 0) printf("role_send(E_TRAJ) failed #%i\n", -ret);

            usleep(1000);
        }
    }

void sendPosServo(eServos s, uint16_t us){
    sMsg msg = {{0}};

    msg.header.destAddr = ADDRI_MAIN_IO;
    msg.header.type = E_SERVOS;
    msg.header.size = 2 + 3;
    msg.payload.servos.nb_servos = 1;
    msg.payload.servos.servos[0].id = s;
    msg.payload.servos.servos[0].us = us;

    bn_send(&msg);

    }

int newSpeed(float speed){
    sMsg msg = {{0}};

    if( fabs(speed) > 150.){
        return -1;
        }

    msg.header.destAddr = role_get_addr(ROLE_PROPULSION);
    msg.header.type = E_SPEED_SETPOINT;
    msg.header.size = sizeof(msg.payload.speedSetPoint);
    msg.payload.speedSetPoint.speed = speed;

    bn_send(&msg);

    return 1;
    }



int sendSeg(const sPt_t *p, const sVec_t *v){ //the robot goes directly to the point or the vector
    sPath_t path;
    sTrajEl_t traj[2]={
        {{0. , 0.},{0. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 0.},
        {{0. , 0.},{0. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 1.}
        };

    if( ((p == NULL) && (v == NULL)) || ((p != NULL) && (v != NULL)) ){
        return -1;
        }

    if(p != NULL){
        traj[0].p1 = obs[0].c;
        traj[0].p2 = *p;
        traj[1].p1 = traj[0].p2;
        traj[1].p2 = traj[0].p2;
        }

    if(v != NULL){
        traj[0].p1 = obs[0].c;
        traj[0].p2.x = obs[0].c.x + v->x;
        traj[0].p2.y = obs[0].c.y + v->y;
        traj[1].p1 = traj[0].p2;
        traj[1].p2 = traj[0].p2;
        }

    path.path = &traj[0];
    path.path_len=2;
    send_robot(path);

    return 1;
    }
