/*
 * obj_com.c
 *
 *  Created on: 18 avr. 2014
 *      Author: seb
 */


#include "obj_com.h"

#include <math.h>
#include "../botNet/shared/botNet_core.h"
#include "obj_types.h"
#include "main.h"

sServo_t listServo[2]={ //TODO other servo
    {SERVO_PRIM_ARM_LEFT,  650, 0, 2400, 180},
    {SERVO_PRIM_ARM_RIGHT, 650, 0, 2400, 180},
    };

void send_robot(sPath_t path){
    sMsg outMsg = {{0}};
    int i, ret;
    last_tid++;
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
            outMsg.payload.traj.tid = last_tid;

            if((ret=role_sendRetry(&outMsg, MAX_RETRIES))<=0){
                printf("role_sendRetry(E_TRAJ) failed #%i\n", -ret);
            }

            usleep(1000);
        }
    }

void stop_robot(void){
    sTrajEl_t traj = {{obs[0].c.x, obs[0].c.y}, {obs[0].c.x, obs[0].c.y}, {{obs[0].c.x, obs[0].c.y}, 0, 0, 1}, 0, 0, 0};
    sPath_t path;

    path.path = &traj;
    path.path_len = 1;
    send_robot(path);
    }

int sendPosServo(eServos s, int16_t us, int16_t a){ // us or a = -1 if no use
    sMsg msg = {{0}};
    sPt_t p1, p2;
    sLin_t l;
    int i=0;

    if( ((us == -1) && (a == -1)) || ((us != -1) && (a != -1)) ){
            return -1;
            }

    if(a != -1){
        i = 0;
        while(s != listServo[i].id){
            i++;
            if( i > sizeof(listServo)/sizeof(*listServo)) break;
            }
        p1.x = listServo[i].a1;
        p1.y = listServo[i].u1;
        p2.x = listServo[i].a2;
        p2.y = listServo[i].u2;
        convPts2Line(&p1, &p2, 0, &l);

        if(l.b == 0) return -1;

        us = -(l.a*a + l.c)/l.b;
        }

    msg.header.destAddr = ADDRI_MAIN_IO;
    msg.header.type = E_SERVOS;
    msg.header.size = 2 + 3;
    msg.payload.servos.nb_servos = 1;
    msg.payload.servos.servos[0].id = s;
    msg.payload.servos.servos[0].us = us;

    bn_sendRetry(&msg, MAX_RETRIES);

    return 1;
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

    if(bn_sendRetry(&msg, MAX_RETRIES)<=0){
        printf("bn_sendRetry(E_SPEED_SETPOINT) failed!\n");
    }

    return 1;
    }

int setPos(sPt_t *p, sNum_t theta){
    sMsg msg = {{0}};
    msg.header.type = E_POS;
    msg.header.size = sizeof(msg.payload.pos);

    msg.payload.pos.id = 0;
    msg.payload.pos.u_a = 0;
    msg.payload.pos.u_a_theta = 0;
    msg.payload.pos.u_b = 0;
    msg.payload.pos.theta = theta;
    msg.payload.pos.x = p->x;
    msg.payload.pos.y = p->y;
    obs[0].c.x = p->x;
    obs[0].c.y = p->y;
    theta_robot = theta;
    _current_pos = obs[0].c;

    if(role_sendRetry(&msg, MAX_RETRIES)<=0){
        printf("bn_sendRetry(E_POS) failed!\n");
    }

    return 0;
    }

int sendSeg(const sPt_t *p, const sVec_t *v){ //the robot goes directly to the point or the vector
    sPath_t path;
    sTrajEl_t t = {{0}};

    if( ((p == NULL) && (v == NULL)) || ((p != NULL) && (v != NULL)) ){
        return -1;
        }

    if(p != NULL){
        t.p1 = obs[0].c;
        t.p2 = *p;
        distPt2Pt(&t.p1, &t.p2, &t.seg_len);

        t.obs.c = t.p2;
        t.obs.r = 0.;
        t.arc_len = 0.;
        t.sid = 0;
        }

    if(v != NULL){
        t.p1 = obs[0].c;
        t.p2.x = t.p1.x + v->x;
        t.p2.y = t.p1.y + v->y;
        distPt2Pt(&t.p1, &t.p2, &t.seg_len);

        t.obs.c = t.p2;
        t.obs.r = 0.;
        t.arc_len = 0.;
        t.sid = 0;
    }

    path.path = &t;
    path.path_len = 1;
    send_robot(path);

    return 1;
    }
