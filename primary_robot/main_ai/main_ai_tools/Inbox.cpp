/*
 * Inbox.cpp
 *
 *  Created on: 15 mai 2015
 *      Author: seb
 */

#include "Inbox.h"


#include "tools.h"
#include "communications.h"
extern "C"{
#include "botNet_core.h"
#include "roles.h"
}


Inbox::Inbox() : addrProp(0) {
    // TODO Auto-generated constructor stub

}

Inbox::~Inbox() {
    // TODO Auto-generated destructor stub
}


/*
 * Check if a new message is available and do the appropriate operation
 */
void Inbox::checkInbox(){
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

            if(!addrProp)
                addrProp = msgIn.header.srcAddr;
            else if(msgIn.header.srcAddr != addrProp){
                logs << ERR << "Receives a new generic pos status of an other source : " << msgIn.header.srcAddr;
                path.stopRobot(true);
                exit(EXIT_FAILURE);
            }
            statuses.receivedNewStatus(msgIn.payload.genericPosStatus);
            break;
        case E_GOAL:
            logs << MES << "[GOAL] robot" << msgIn.payload.genericPosStatus.id << "@(" << msgIn.payload.genericPosStatus.pos.x << ", " << msgIn.payload.genericPosStatus.pos.y << ", " << msgIn.payload.genericPosStatus.pos.theta * 180. / M_PI << ")";

            posGoal.x = msgIn.payload.genericPosStatus.pos.x;
            posGoal.y = msgIn.payload.genericPosStatus.pos.y;
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
bool Inbox::lastGoal(Point2D<float>& goal){
    if(!posGoal.x && !posGoal.y)
        return false;

    goal = posGoal;
    posGoal = {0, 0};
    return true;
}
