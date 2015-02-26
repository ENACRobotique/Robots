/*
 * net.cpp
 *
 *  Created on: 24 févr. 2015
 *      Author: seb
 */

#include <net.h>

#include <stdint.h>
#include <cmath>
#include <iostream>

extern "C"{
#include "roles.h"
}
#include "messages.h"

#define round(a) ((a - (int) a) < 0.5 ? (a) : (a+1))
#define conv(a, b) (int) round(((double) a * pow(2,b)))

unsigned int tid = 0; //the same tid is used by all path object

Net::Net() {
    // TODO Auto-generated constructor stub

}

Net::~Net() {
    // TODO Auto-generated destructor stub
}

void Net::maintenace(){

    sendPath2Net();

}

/*
 * Add a new path in the list of path ready to send
 */
void Net::sendPath(vector <sTrajEl_t> &trajEl){
    queue <sTrajEl_t> empty;
    swap(_trajEl, empty);

    for(sTrajEl_t i : trajEl){
        _trajEl.push(i);
    }
}


/*
 * Create a new message E_TRAJ_ORIENT_EL with a list elements of trajectory define by sTrajEl_t and send it.
 */
void Net::sendPath2Net(){
    sMsg outMsg;
    int ret;

    if(!_trajEl.empty()){
        outMsg.header.type = E_TRAJ_ORIENT_EL;
        outMsg.header.size = sizeof(outMsg.payload.trajOrientEl);

        for(unsigned int i = 0 ; i < _trajEl.size(); i++){
            for(unsigned int j = 0 ; j < 2 ; j++){
                outMsg.payload.trajOrientEl.elts[j].p1_x = conv(_trajEl.back().p1.x, 6);
                outMsg.payload.trajOrientEl.elts[j].p1_y = conv(_trajEl.back().p1.y, 6);
                outMsg.payload.trajOrientEl.elts[j].p2_x = conv(_trajEl.back().p2.x, 6);
                outMsg.payload.trajOrientEl.elts[j].p2_x = conv(_trajEl.back().p2.y, 6);

                outMsg.payload.trajOrientEl.elts[j].c_x = conv(_trajEl.back().obs.c.x, 6);
                outMsg.payload.trajOrientEl.elts[j].c_y = conv(_trajEl.back().obs.c.y, 6);
                outMsg.payload.trajOrientEl.elts[j].c_r = conv(_trajEl.back().obs.r, 6);

                outMsg.payload.trajOrientEl.elts[j].theta1 =  0; //TODO
                outMsg.payload.trajOrientEl.elts[j].theta2 =  0; //TODO

                outMsg.payload.trajOrientEl.elts[j].seg_len = conv(_trajEl.back().seg_len, 5);
                outMsg.payload.trajOrientEl.elts[j].rot1_dir =  0; //TODO
                outMsg.payload.trajOrientEl.elts[j].arc_len = conv(_trajEl.back().arc_len, 5);
                outMsg.payload.trajOrientEl.elts[j].rot2_dir = 0; //TODO

                _trajEl.pop();
            }

            outMsg.payload.trajOrientEl.t1 = 0; //TODO define reference time and add a delta (time to send the message)
            outMsg.payload.trajOrientEl.t2 = 0; //TODO
            outMsg.payload.trajOrientEl.t3 = 0; //TODO
            outMsg.payload.trajOrientEl.t4 = 0; //TODO

            outMsg.payload.trajOrientEl.tid = tid;
            outMsg.payload.trajOrientEl.sid = i;

            if ((ret = role_sendRetry(&outMsg, MAX_RETRIES)) <= 0) {
                printf("[ERROR] [%s:%i] : role_sendRetry(E_TRAJ) failed #%i\n", __FILE__, __LINE__, -ret);
            }
            cout << "[INFO] A new path was send" << endl;
        }

    tid++;
    }

}

/*//send a E_TRAJ

         if (!_path.empty()){
            for (unsigned int i = 0; i < _path_len; i++) {
                printElTraj(i);

                outMsg.header.type = E_TRAJ;
                outMsg.header.size = sizeof(outMsg.payload.traj);

                outMsg.payload.traj.p1_x = _path[i].p1.x;
                outMsg.payload.traj.p1_y = _path[i].p1.y;
                outMsg.payload.traj.p2_x = _path[i].p2.x;
                outMsg.payload.traj.p2_y = _path[i].p2.y;
                outMsg.payload.traj.seg_len = _path[i].seg_len;

                outMsg.payload.traj.c_x = _path[i].obs.c.x;
                outMsg.payload.traj.c_y = _path[i].obs.c.y;
                outMsg.payload.traj.c_r = _path[i].obs.r;
                outMsg.payload.traj.arc_len = _path[i].arc_len;

                outMsg.payload.traj.sid = i;
                outMsg.payload.traj.tid = tid;

                if ((ret = role_sendRetry(&outMsg, MAX_RETRIES)) <= 0) {
                    printf("[ERROR] [path.cpp] : role_sendRetry(E_TRAJ) failed #%i\n", -ret);
                }
                cout << "[INFO] A new path was send" << endl;

                usleep(1000);
            }
        }
        delete path.path; //delete the previous path send;

        path.dist = _dist;
        path.path_len = _path_len;
        path.path = new sTrajEl_t[_path_len];
        for(unsigned int i = 0 ; i < _path_len ; i++){
            path.path[i] = _path[i];
        }


 */
