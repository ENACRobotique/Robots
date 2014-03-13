/*
 * bn_testFunc.c
 *
 *  Created on: 16 janv. 2014
 *      Author: quentin
 */


/*
 * useful test functions for botNet.
 */

#include "bn_testFunc.h"
#include "../../../../tools/libraries/Timeout/timeout.h"
#include "../../../../global_errors.h"
#include "node_cfg.h"

#include <stdlib.h>
#include <inttypes.h>

enum {
    E_CBR_START,
    E_CBR_START_ACK,
    E_CBR_STOP,
    E_CBR_STATS,
    E_CBR_RESET
};


enum {
    E_WELL_STATS,
    E_WELL_RESET
};


typedef struct sFluxDescDef{
    bn_Address receiver;
    int8_t fluxID;
    uint8_t ack;            // boolean
    uint32_t period;        // period of sending (microsecond)
    uint32_t sw;            // memory to store a stopwatch
    uint8_t size;           // size of the paylod of the packets
    uint32_t numberLeft;    // amount of packet left
    uint32_t numberSend;    // number of message correctly send (and acked if it was asked)
    uint32_t numberNSend;   // number of message incorrectly send (and/or Nacked)
    struct sFluxDescDef *next;
} sFluxDescriptor;

typedef struct sStatDescDef{
    bn_Address sender;
    int8_t fluxID;
    uint32_t sw;            // memory to store a stopwatch
    uint32_t meanIAT;       // mean inter-arrival time (µs)
    uint32_t numberRX;      // number of message received
    struct sStatDescDef *next;
} sStatDescriptor;




// flux database (chained list, pointer to first elem)
sFluxDescriptor *fluxDB=0;

//stat database
sStatDescriptor *statDB=0;


/* Creates an new flux.
 * Must be executed on the source.
 */
int cbr_newFlux(sMsg *msg){
    sFluxDescriptor *curP, *prevP=0;

    if ( msg->payload.CBRCtrl.fluxID==-1 ) return -ERR_OUT_OF_BOND;

    //locate end of db
    for ( curP=fluxDB ; curP!=0 && curP->fluxID!=msg->payload.CBRCtrl.fluxID ; curP=curP->next) prevP=curP;
    if ( curP!=0) return -ERR_ALREADY_IN_USE;

    //malloc
    curP=malloc(sizeof(sFluxDescriptor));
    if ( curP==0 ) return -ERR_INSUFFICIENT_MEMORY;
    prevP->next=curP;

    //fill informations
    curP->receiver=msg->payload.CBRCtrl.dest;
    curP->fluxID=msg->payload.CBRCtrl.fluxID;
    if (msg->payload.CBRCtrl.flag==E_CBR_START_ACK) curP->ack=1;
    else curP->ack=0;
    curP->period=msg->payload.CBRCtrl.period;
    curP->sw=0;
    curP->size=msg->payload.CBRCtrl.size;
    curP->numberLeft=msg->payload.CBRCtrl.number;
    curP->numberNSend=0;
    curP->numberSend=0;
    curP->next=0;

    return 0;
}

int cbr_deleteAll(){
    sFluxDescriptor *curP, *nextP;
    for ( curP=fluxDB ; curP!=0 ; curP=nextP){
        nextP=curP->next;
        free(curP);
    }
    fluxDB=0;
    return 0;
}

int cbr_findAndKill(sMsg * msg){
    sFluxDescriptor *curP;

    if (msg->payload.CBRCtrl.fluxID == -1){
        for ( curP=fluxDB ; curP!=0  ; curP=curP->next) curP->numberLeft=0;
        return 0;
    }

    for ( curP=fluxDB ; curP!=0 && curP->fluxID!=msg->payload.CBRCtrl.fluxID ; curP=curP->next);
    if ( curP == 0 ) return -ERR_NOT_FOUND;
    curP->numberLeft=0;
    return 0;
}


int cbr_printStat(sMsg *msg){
    sFluxDescriptor *curP;
    int i=0;

    //every flux
    if ( msg->payload.CBRCtrl.fluxID==-1 ){
        bn_printfDbg("Full CBR report from x%hx|i%hx|u%hx\n", MYADDRX, MYADDRI, MYADDRU);
        for ( curP=fluxDB ; curP!=0 ; curP=curP->next){
            bn_printfDbg("id %"PRIi8", to %"PRIx16", send %"PRIu32", Nsend %"PRIu32"\n", curP->fluxID, curP->receiver, curP->numberSend, curP->numberNSend);
            i++;
        }
        bn_printfDbg("End CBR report. nb entries : %d",i);
    }
    else {
        for ( curP=fluxDB ; curP!=0 && curP->fluxID!=msg->payload.CBRCtrl.fluxID ; curP=curP->next){
            bn_printfDbg("short CBR report from x%hx|i%hx|u%hx\n", MYADDRX, MYADDRI, MYADDRU);
            bn_printfDbg("id %"PRIi8", to %"PRIx16", send %"PRIu32", Nsend %"PRIu32"\n", curP->fluxID, curP->receiver, curP->numberSend, curP->numberNSend);
        }
    }

    return 0;
}

// constant bit rate source control message handler
// Should receive any  E_CBR_CTRL messages.
int cbr_controller(sMsg *msg){

    // check type
    if ( msg->header.type!=E_CBR_CTRL) return -ERR_BN_WRONG_TYPE;

    // switch on the flag of the message
    switch ( msg->payload.CBRCtrl.flag){
    case E_CBR_START :
    case E_CBR_START_ACK :
        cbr_newFlux(msg);
        break;
    case E_CBR_RESET :
        cbr_deleteAll();
        break;
    case E_CBR_STOP :
        cbr_findAndKill(msg);
        break;
    case E_CBR_STATS :
        cbr_printStat(msg);
        break;
    default : return -ERR_WRONG_FLAG;
    }
    return 0;
}

/* Deamon managing the sending of all the messages.
 * Must run on the source
 */
int cbr_deamon(){
    sFluxDescriptor *curP;
    int retval;

    //prepare the test packet
    sMsg testPkt;
    testPkt.header.type=E_TEST_PKT;

    //for every entry
    for ( curP=fluxDB ; curP!=0 ; curP=curP->next){

        //check amount of messages left
        if ( curP!=0 ){

            //check stopwatch
            if ( stopwatch(&(curP->sw)) >= curP->period ){

                //restart it
                curP->sw=0;
                stopwatch(&(curP->sw));

                //fill the pkt with this flux data
                testPkt.header.destAddr=curP->receiver;
                testPkt.header.size=curP->size;
                testPkt.payload.testMsg.fluxID=curP->fluxID;

                //send message according to ack field
                if ( curP->ack ) retval=bn_sendAck(&testPkt);
                else retval=bn_send(&testPkt);

                //log result
                if ( retval<0 ) curP->numberNSend++;
                else curP->numberSend++;

                curP->numberLeft--;
            }
        }
    }
    return 0;
}

/* sends a "CBR start" message to server
 * Arguments :
 *  flux >=0
 *  period in µs
 */
int cbr_start(bn_Address server, bn_Address receiver,uint8_t fluxID, uint32_t period, uint8_t size,uint32_t number){
    sMsg msg;
    msg.header.destAddr=server;
    msg.header.type=E_CBR_CTRL;
    msg.header.size=sizeof(sCBRCtrl);

    msg.payload.CBRCtrl.dest=receiver;
    msg.payload.CBRCtrl.flag=E_CBR_START;
    msg.payload.CBRCtrl.fluxID=fluxID;
    msg.payload.CBRCtrl.number=number;
    msg.payload.CBRCtrl.period=period;
    msg.payload.CBRCtrl.size=size;

    return bn_send(&msg);
}

/* sends a "CRB start with acknowledgments" message to server
 *
 */
int cbrAcked_start(bn_Address server, bn_Address receiver,uint8_t fluxID, uint32_t period, uint8_t size,uint32_t number){
    sMsg msg;
    msg.header.destAddr=server;
    msg.header.type=E_CBR_CTRL;
    msg.header.size=sizeof(sCBRCtrl);

    msg.payload.CBRCtrl.dest=receiver;
    msg.payload.CBRCtrl.flag=E_CBR_START_ACK;
    msg.payload.CBRCtrl.fluxID=fluxID;
    msg.payload.CBRCtrl.number=number;
    msg.payload.CBRCtrl.period=period;
    msg.payload.CBRCtrl.size=size;

    return bn_send(&msg);
}

/* Sends a "print sender result" to server for fluxID.
 * fluxID==-1 means every flux
 */
int cbr_printResults(bn_Address server, int8_t fluxID){
    sMsg msg;
    msg.header.destAddr=server;
    msg.header.type=E_CBR_CTRL;
    msg.header.size=sizeof(sCBRCtrl);

    msg.payload.CBRCtrl.flag=E_CBR_STATS;
    msg.payload.CBRCtrl.fluxID=fluxID;

    return bn_send(&msg);
}

/* Sens a "stops any current flux and delete your db" message to server.
 *
 */
int cbr_reset(bn_Address server){
    sMsg msg;
    msg.header.destAddr=server;
    msg.header.type=E_CBR_CTRL;
    msg.header.size=sizeof(sCBRCtrl);

    msg.payload.CBRCtrl.flag=E_CBR_RESET;

    return bn_send(&msg);
}

/* sends an "stop fluxID" message to server
 * Arguments :
 *  fluxID == -1 mean every flux
 *
 */
int cbr_stop(bn_Address server, int8_t fluxID){
    sMsg msg;
    msg.header.destAddr=server;
    msg.header.type=E_CBR_CTRL;
    msg.header.size=sizeof(sCBRCtrl);

    msg.payload.CBRCtrl.flag=E_CBR_STOP;
    msg.payload.CBRCtrl.fluxID=fluxID;

    return bn_send(&msg);
}

int well_recordPkt(sMsg *msg){
    sStatDescriptor *curP=0, *prevP=statDB;

    //record global stat

    //search for the duet (sender,fluxID) in the DB and update its value
    for (curP=statDB; curP!=0 && (msg->header.srcAddr==curP->sender && msg->payload.testMsg.fluxID==curP->fluxID) ; curP=curP->next){
        prevP=curP;
    }
    //if found, update stat values
    if ( curP!=0 ) {
        //increment number of pkts
        curP->numberRX++;

        //compute  man store mean IAT
        curP->meanIAT=( stopwatch(&(curP->sw))*(curP->numberRX - 1) + curP->meanIAT)/curP->numberRX;

        //restart stopwatch
        curP->sw=0;
        stopwatch(&(curP->sw));
    }
    //if not found, create new entry
    else {

        //allocate memory
        if ( (curP=malloc(sizeof(sStatDescriptor)))<0) return -ERR_INSUFFICIENT_MEMORY;
        prevP->next=curP;

        //fill it
        curP->fluxID=msg->payload.testMsg.fluxID;
        curP->meanIAT=0;
        curP->next=0;
        curP->numberRX=1;
        curP->sender=msg->header.srcAddr;
        curP->sw=0;
        stopwatch(&(curP->sw));
    }
    return 0;
}

/* Replies to the sender of msg by the asked statistics
 *
 */
int well_sendStat(sMsg *msg){
    sStatDescriptor *curP;
    int i=0;

    //every flux
    if ( msg->payload.wellCrtl.fluxID==-1 && msg->payload.wellCrtl.src==0 ){
        bn_printfDbg("Full Well report from x%"PRIx16"|i%"PRIx16"|u%"PRIx16"\n", MYADDRX, MYADDRI, MYADDRU);
        for ( curP=statDB ; curP!=0 ; curP=curP->next){
            i++;
        }
        bn_printfDbg("End Well report. nb entries : %d",i);
    }
    else {
        bn_printfDbg("short Well report from x%"PRIx16"|i%"PRIx16"|u%"PRIx16"\n", MYADDRX, MYADDRI, MYADDRU);

        for ( curP=statDB ; curP!=0 ; curP=curP->next){
            if ( (msg->payload.wellCrtl.src==0 || msg->payload.wellCrtl.src==curP->sender) \
                   && (msg->payload.wellCrtl.fluxID==-1 || msg->payload.wellCrtl.fluxID==curP->fluxID) ){
                bn_printfDbg("from %"PRIx16", id %"PRIi8", rxed %"PRIu32", IAT %"PRIu32"\n", curP->sender, curP->fluxID, curP->numberRX, curP->meanIAT);
            }
        }
    }

    return 0;
}

/* Deletes all stat
 *
 */
int well_deleteAll(){
    sStatDescriptor *curP,*nextP;
    for (curP=statDB; curP!=0 ; ){
        nextP=curP->next;
        free(curP);
        curP=nextP;
    }
    return 0;
}


/* Stat generator on the receiver side.
 * Should receive any  E_TEST_PKT and E_WELL_CTRL messages.
 */
// counts received test packets
int well_deamon(sMsg *msg){

    //check type
    if (msg->header.type!=E_TEST_PKT && msg->header.type!=E_WELL_CTRL) return -ERR_BN_WRONG_TYPE;

    switch (msg->header.type){
    case E_TEST_PKT :
         return well_recordPkt(msg);
        break;
    case E_WELL_CTRL :
        if (msg->payload.wellCrtl.flag==E_WELL_RESET) return well_deleteAll();
        else if (msg->payload.wellCrtl.flag==E_WELL_STATS)return well_sendStat(msg);
        break;
    default :
        break;
    }
    return 0;
}

/* sends a result request
 * Arguments :
 *  server : address of the node where the resuslts are recorded
 *  sender : source of the traffic monitored (0 for all sender)
 *  fluxID : ID of the flux monitored (-1 for all flux of sender specified with "sender" argument)
 */
int well_requestStat(bn_Address server,bn_Address sender, int8_t fluxID){
    sMsg msg;

    msg.header.destAddr=server;
    msg.header.size=sizeof(sWellCtrl);
    msg.header.type=E_WELL_CTRL;

    msg.payload.wellCrtl.flag=E_WELL_STATS;
    msg.payload.wellCrtl.fluxID=fluxID;
    msg.payload.wellCrtl.src=sender;

    return bn_send(&msg);
}

// sends a "delete everything" request
int well_reset(bn_Address server){
    sMsg msg;

    msg.header.destAddr=server;
    msg.header.size=sizeof(sWellCtrl);
    msg.header.type=E_WELL_CTRL;

    msg.payload.wellCrtl.flag=E_WELL_RESET;

    return bn_send(&msg);
}

