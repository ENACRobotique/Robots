/*
 * obj_com.c
 *
 *  Created on: 18 avr. 2014
 *      Author: seb
 */


#include "obj_com.h"

void send_robot(sPath_t path){
    sMsg outMsg;
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
