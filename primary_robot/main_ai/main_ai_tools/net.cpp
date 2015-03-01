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
#include <unistd.h>
#include "roles.h"
}
#include "messages.h"

#define TRAJ_ORIENT 1 //1 active else 0

#define ROUND(a) ((a - (int) a) < 0.5 ? (a) : (a+1))
#define CONV2TRAJORIENT(a, b) (int) ROUND(((double) a) * pow(2,b))
#define CONV2TRAJ(a, b) (((double) a)/(pow(2,b)))

unsigned int tid = 0; //the same tid is used by all path object

Net::Net() {
    // TODO Auto-generated constructor stub

}

Net::~Net() {
    // TODO Auto-generated destructor stub
}

void Net::maintenace(){

#if TRAJ_ORIENT
    convTrajToTrajOrient();
    sendPathOrientToNet();
#else
    sendPathToNet();
#endif

}

/*
 * Clear _trajEl and _trajOrientEl
 */
void Net::clearEl(){//TODO warning if no empty
    queue <sTrajEl_t> empty;
    swap(_trajEl, empty);

    queue <sTrajOrientEl_t> emptyOrient;
    swap(_trajOrientEl, emptyOrient);
}

/*
 * Add a new path in the list of path ready to send
 */
void Net::sendPath(vector <sTrajEl_t> &trajEl){
    clearEl();

    for(sTrajEl_t i : trajEl){
        _trajEl.push(i);
    }
}

/*
 * Add a new path orient in the list of path ready to send
 */
void Net::sendPathOrient(vector <sTrajOrientEl_t> &trajElOrient){
    clearEl();

    for(sTrajOrientEl_t i : trajElOrient){
        _trajOrientEl.push(i);
    }
}

/*
 * Transfers _trajEl to _trajOrientEl and compute the new parameter.
 */
void Net::convTrajToTrajOrient(){
    sTrajOrientEl_t trajOrient;

    while(!_trajEl.empty()){
        trajOrient.t1 = 0; //TODO
        trajOrient.t2 = 0; //TODO

        trajOrient.p1 = _trajEl.front().p1;
        trajOrient.p2 = _trajEl.front().p2;

        trajOrient.obs = _trajEl.front().obs;

        trajOrient.theta1 = 0; //TODO get current theta
        trajOrient.theta2 = 0; //TODO get current theta

        trajOrient.seg_len = _trajEl.front().seg_len;
        trajOrient.arc_len = _trajEl.front().arc_len;

        trajOrient.rot1_dir = 0; //TODO
        trajOrient.rot2_dir  = 0; //TODO

        _trajEl.pop();
        _trajOrientEl.push(trajOrient);
    }

}

/*
 * Sends the path save in _trajEl to the prop with a E_TRAJ message
 */
void Net::sendPathToNet(){
    sMsg outMsg;
    int ret;

    if(!_trajEl.empty()){
        unsigned int size = _trajEl.size();

        outMsg.header.type = E_TRAJ;
        outMsg.header.size = sizeof(outMsg.payload.traj);

        outMsg.payload.traj.tid = tid;

        for (unsigned int i = 0; i < size; i++) {
            outMsg.header.type = E_TRAJ;
            outMsg.header.size = sizeof(outMsg.payload.traj);
            outMsg.payload.traj.sid = i;

            outMsg.payload.traj.p1_x = _trajEl.front().p1.x;
            outMsg.payload.traj.p1_y = _trajEl.front().p1.y;
            outMsg.payload.traj.p2_x = _trajEl.front().p2.x;
            outMsg.payload.traj.p2_y = _trajEl.front().p2.y;
            outMsg.payload.traj.seg_len = _trajEl.front().seg_len;

            outMsg.payload.traj.c_x = _trajEl.front().obs.c.x;
            outMsg.payload.traj.c_y = _trajEl.front().obs.c.y;
            outMsg.payload.traj.c_r = _trajEl.front().obs.r;
            outMsg.payload.traj.arc_len = _trajEl.front().arc_len;

            _trajEl.pop();

            if ((ret = role_sendRetry(&outMsg, MAX_RETRIES)) <= 0) {
                printf("[ERROR] [path.cpp] : role_sendRetry(E_TRAJ) failed #%i\n", -ret);
            }else
                cout << "[INFO] A new path was send" << endl;

            usleep(1000);
        }
    tid++;
    }
 }


/*
 * Sends the path orient save in _trajOrientEl to the prop with a E_TRAJ_ORIENT_EL message
 */
void Net::sendPathOrientToNet(){
    sMsg outMsg;
    int ret, i = 0;

    if(!_trajOrientEl.empty()){
        outMsg.header.type = E_TRAJ_ORIENT_EL;
        outMsg.header.size = sizeof(outMsg.payload.trajOrientEl);

        while(!_trajOrientEl.empty()){
            for(unsigned int j = 0 ; j < 2 ; j++){
                outMsg.payload.trajOrientEl.elts[j].p1_x = CONV2TRAJORIENT(_trajOrientEl.front().p1.x, 6);
                outMsg.payload.trajOrientEl.elts[j].p1_y = CONV2TRAJORIENT(_trajOrientEl.front().p1.y, 6);

                outMsg.payload.trajOrientEl.elts[j].p2_x = CONV2TRAJORIENT(_trajOrientEl.front().p2.x, 6);
                outMsg.payload.trajOrientEl.elts[j].p2_y = CONV2TRAJORIENT(_trajOrientEl.front().p2.y, 6);

                outMsg.payload.trajOrientEl.elts[j].c_x = CONV2TRAJORIENT(_trajOrientEl.front().obs.c.x, 6);
                outMsg.payload.trajOrientEl.elts[j].c_y = CONV2TRAJORIENT(_trajOrientEl.front().obs.c.y, 6);
                outMsg.payload.trajOrientEl.elts[j].c_r = CONV2TRAJORIENT(_trajOrientEl.front().obs.r, 6);

                outMsg.payload.trajOrientEl.elts[j].theta1 =  CONV2TRAJORIENT(_trajOrientEl.front().theta1, 13);
                outMsg.payload.trajOrientEl.elts[j].theta2 =  CONV2TRAJORIENT(_trajOrientEl.front().theta2, 13);

                outMsg.payload.trajOrientEl.elts[j].seg_len = CONV2TRAJORIENT(_trajOrientEl.front().seg_len, 5);
                outMsg.payload.trajOrientEl.elts[j].rot1_dir =  _trajOrientEl.front().rot1_dir;
                outMsg.payload.trajOrientEl.elts[j].arc_len = CONV2TRAJORIENT(_trajOrientEl.front().arc_len, 5);
                outMsg.payload.trajOrientEl.elts[j].rot2_dir = _trajOrientEl.front().rot2_dir;

                _trajOrientEl.pop();
                if(_trajOrientEl.empty())
                    break;
            }

            outMsg.payload.trajOrientEl.t = 0; //TODO define reference time and add a delta (time to send the message)
            outMsg.payload.trajOrientEl.dt1 = 0; //TODO
            outMsg.payload.trajOrientEl.dt2 = 0; //TODO
            outMsg.payload.trajOrientEl.dt3 = 0; //TODO

            outMsg.payload.trajOrientEl.tid = tid;
            outMsg.payload.trajOrientEl.sid = i;

            if ((ret = role_sendRetry(&outMsg, MAX_RETRIES)) <= 0) {
                printf("[ERROR] [%s:%i] : role_sendRetry(E_TRAJ_ORIENT_EL) failed #%i\n", __FILE__, __LINE__, -ret);
            }else
                cout << "[INFO] A new path orient was send" << endl;

            i++;

            usleep(1000);
        }

    tid++;
    }
}
