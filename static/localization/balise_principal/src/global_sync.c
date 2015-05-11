/*
 * global_sync.c
 *
 *  Created on: 10 mai 2015
 *      Author: quentin
 */

#include "global_sync.h"
#include "global_errors.h"
#include "bn_intp.h"
#include "stddef.h"
#include "roles.h"
#include "string.h"

typedef struct{
    bn_Address queryOrigin;
    eSyncType type :8;
    union{
        bn_Address addr;
        uint8_t role;
    };
    eSyncStatus status;
}syncQueryStatus;

syncQueryStatus queryArray[SYNC_ARRAY_SIZE];    // circular buffer
syncQueryStatus queryBeacons = {0};
int gs_wIndex=0; // write index (where we will write next time)
int gs_rIndex=0; // read index (which data should we handle next time)
int gs_oIndex=0; // oldest data index (beginning of our circular buffer)
int gs_size=0;

void gs_receiveQuery(sMsg *msg){
    int i=0;

    if (msg->header.type == E_SYNC_QUERY){
        while (i<msg->payload.syncQuery.nb && gs_size<SYNC_ARRAY_SIZE){
            queryArray[gs_wIndex].queryOrigin = msg->header.srcAddr;
            queryArray[gs_wIndex].type = msg->payload.syncQuery.cfgs[i].type;
            if (queryArray[gs_wIndex].type == SYNCTYPE_ADDRESS){
                queryArray[gs_wIndex].addr = msg->payload.syncQuery.cfgs[i].addr;
            }
            else if (queryArray[gs_wIndex].type == SYNCTYPE_ROLE){
                queryArray[gs_wIndex].role = msg->payload.syncQuery.cfgs[i].role;
            }
            if (queryArray[gs_wIndex].type == SYNCTYPE_BEACONS){
                queryBeacons.queryOrigin = msg->header.srcAddr;
                queryBeacons.type = SYNCSTATUS_TODO;
            }
            else {
                queryArray[gs_wIndex].status = SYNCSTATUS_TODO;
                gs_wIndex++;
                gs_wIndex %= SYNC_ARRAY_SIZE;
                gs_size ++;
            }
            i++;
        }
        if  (i<msg->payload.syncQuery.nb && gs_size == SYNC_ARRAY_SIZE){
            sMsg reply;
            reply.header.destAddr = msg->header.srcAddr;
            reply.header.type = E_SYNC_RESPONSE;
            reply.header.size = sizeof(reply.payload.syncResponse.cfgs[0]) * (msg->payload.syncQuery.nb-i);
            int k=0;
            while (i<msg->payload.syncQuery.nb && k<18){
                reply.payload.syncResponse.cfgs[k].type = msg->payload.syncQuery.cfgs[i].type;
                if (reply.payload.syncResponse.cfgs[k].type == SYNCTYPE_ADDRESS){
                    reply.payload.syncResponse.cfgs[k].addr = msg->payload.syncQuery.cfgs[i].addr;
                }
                else if (reply.payload.syncResponse.cfgs[k].type == SYNCTYPE_ROLE){
                    reply.payload.syncResponse.cfgs[k].role = msg->payload.syncQuery.cfgs[i].role;
                }
                k++;
                i++;
            }
            bn_send(&reply);
        }
    }
}

/* gs_testOne : performs the required tests for global sync for the next requested element and sends reports when appropriate (i.e. on error or when every test has been performed)
 * Return value :
 *  0 if nothing to do
 *  <0 if error
 */
int gs_testOne(){
    int ret=0;
    // next element of the circular buffer (size not null)
    if (gs_size && gs_rIndex!=gs_wIndex){
        bn_Address dest=0;
        if (queryArray[gs_rIndex].type == SYNCTYPE_ROLE){
            dest = role_get_addr(queryArray[gs_rIndex].role);
        }
        else if (queryArray[gs_rIndex].type == SYNCTYPE_ADDRESS){
            dest = role_get_addr(queryArray[gs_rIndex].addr);
        }
        // synchro (intp is also used as ping)
        ret = bn_intp_sync(dest,3);
        if (ret>0){
            queryArray[gs_rIndex].status = SYNCSTATUS_OK;
        }
        else if ( ret==-ERR_TRY_AGAIN){
            queryArray[gs_rIndex].status = SYNCSTATUS_SYNC_KO;
        }
        else {
            queryArray[gs_rIndex].status = SYNCSTATUS_PING_KO;
        }
        gs_rIndex = (gs_rIndex+1)%SYNC_ARRAY_SIZE;
    }
    if (ret<0 || gs_rIndex==gs_wIndex){
        // sent result on first error or if everything has been tested
        sMsg msg;
        msg.header.destAddr = queryArray[gs_oIndex].queryOrigin;
        msg.header.type = E_SYNC_RESPONSE;
        msg.payload.syncResponse.nb = 0;
        while (queryArray[gs_oIndex].queryOrigin == msg.header.destAddr  && gs_oIndex < gs_rIndex){
            msg.payload.syncResponse.cfgs[msg.payload.syncResponse.nb].type = queryArray[gs_oIndex].type;
            if (queryArray[gs_oIndex].type == SYNCTYPE_ADDRESS) {
                msg.payload.syncResponse.cfgs[msg.payload.syncResponse.nb].addr = queryArray[gs_oIndex].addr;
            }
            else if (queryArray[gs_oIndex].type == SYNCTYPE_ROLE){
                msg.payload.syncResponse.cfgs[msg.payload.syncResponse.nb].role = queryArray[gs_oIndex].role;
            }
            gs_oIndex = (gs_oIndex+1)%SYNC_ARRAY_SIZE;
            gs_size--;
        }
        msg.header.size = msg.payload.syncResponse.nb*sizeof(msg.payload.syncResponse.cfgs[0]);
        while (bn_sendAck(&msg)<0); // critical, Infinite loop
    }
    return 0;
}

void gs_beaconStatus(eSyncStatus status){
    // Just notify the sender.
    sMsg msg;
    if (queryBeacons.queryOrigin!=0){
        msg.header.destAddr = queryBeacons.queryOrigin;
        msg.header.type = E_SYNC_RESPONSE;
        msg.header.size = sizeof(msg.payload.syncResponse.cfgs[0]);
        msg.payload.syncResponse.nb = 1;
        msg.payload.syncResponse.cfgs[0].type = SYNCTYPE_BEACONS;
        msg.payload.syncResponse.cfgs[0].status = status;
        while (bn_sendAck(&msg)<0); // critical, Infinite loop
        memset(&queryBeacons,0,sizeof(queryBeacons));
    }
}

/* gs_isBeaconRequested : must be tested. If returns 1, the turret MUST start synchronization with beacons
 * Rreturned value :
 *  1 if beacon synchronization is requested and not performed
 *  0 otherwise
 */
int gs_isBeaconRequested(){
    return (queryBeacons.addr != 0 && queryBeacons.status==SYNCSTATUS_TODO?1:0);
}

bn_Address gs_getBeaconQueryOrigin(){
    return queryBeacons.queryOrigin;
}


