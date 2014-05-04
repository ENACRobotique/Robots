/*
 * lib_sync_turret.cpp
 *
 *  Created on: 18 mars 2014
 *      Author: quentin
 */

#include "lib_sync_turret.h"
#include "../../../communication/botNet/shared/botNet_core.h"
#include "../../../communication/network_tools/bn_debug.h"
#include "network_cfg.h"
#include "Arduino.h"
#include "lib_domitille.h"
#include "params.h"

int sync_beginElection(bn_Address addr){
    sMsg outMsg;
    int ret=0;
    outMsg.header.type=E_SYNC_DATA;
    outMsg.header.size=sizeof(sSyncPayload);
    outMsg.payload.sync.flag=SYNCF_BEGIN_ELECTION;
    outMsg.header.destAddr=addr;

    if ((ret=bn_sendAck(&outMsg))<0) {
#ifdef DEBUG_SYNC
        //FIXME : better handle of beacon not available
        bn_printfDbg((char *)"addr %hx offline %d\n",addr,ret);
#endif
        return ret;
    }

    return 1;
}

int sync_sendData(bn_Address addr){
    sMsg outMsg;
    //sends the data message
    outMsg.header.type=E_SYNC_DATA;
    outMsg.header.size=sizeof(sSyncPayload);
    outMsg.header.destAddr=addr;
    outMsg.payload.sync.flag=SYNCF_MEASURES;
    outMsg.payload.sync.index=domi_nbTR();
    outMsg.payload.sync.lastTurnDate=domi_lastTR();
    outMsg.payload.sync.period=domi_lastPeriod();

    return bn_send(&outMsg);
}

int sync_sendEnd(bn_Address addr){
    sMsg outMsg;
    //sends the data message
    outMsg.header.type=E_SYNC_DATA;
    outMsg.header.size=sizeof(sSyncPayload);
    outMsg.header.destAddr=addr;
    outMsg.payload.sync.flag=SYNCF_END_MEASURES;
    outMsg.payload.sync.index=domi_nbTR();
    outMsg.payload.sync.lastTurnDate=domi_lastTR();
    outMsg.payload.sync.period=domi_lastPeriod();

    return bn_sendAck(&outMsg);
}
