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
int gs_wIndex=0; // write index
int gs_rIndex=0; // read index
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
            gs_wIndex++;
            gs_wIndex %= SYNC_ARRAY_SIZE;
            gs_size ++;
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
int gs_loop(){

    // loop over every untested element of the array
        // ping
        // synchro
        // sent result on first error

    return -ERR_MOAERR;
}
