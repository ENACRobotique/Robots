/*
 * bn_ntpi.c : botnet's Network Time Protocol Inverted (master-slave instead of client-server in plain NTP)
 *
 *  Created on: 2 mai 2014
 *      Author: quentin
 */

#include "bn_intp.h"

#include "../../../tools/libraries/Timeout/timeout.h"

#ifdef ARCH_328P_ARDUINO
#include "Arduino.h"
#elif defined(ARCH_X86_LINUX)
#include "../../../core/linux/libraries/Millis/millis.h"
#elif defined(ARCH_LPC21XX)
#include <sys_time.h>
#else
#error "no ARCH_XXX symbol defined"
#endif

int32_t bn_intp_MicrosOffset=0;
int32_t bn_intp_minDT=0;        // T_i - T_i-1  associated to the above value, 0 if no synchro has been measured.


/* bn_intpMsgHandle : handles synchronization messages from master.
 * MUST be bn_attached on every slave to messages of type E_INTP (done in bn_intp_install)
 *
 */
void bn_intp_msgHandle(sMsg *msg){
    static uint8_t prevMessageIndex=0;
    static uint32_t prevMessageDate=0;
    uint32_t receivedDate=micros();

    uint32_t tempDT=msg->payload.intp.time-msg->payload.intp.prevTime;

    if (prevMessageIndex==(msg->payload.intp.index-1) && msg->payload.intp.time-msg && msg->payload.intp.prevTime){
        if (bn_intp_minDT==0 || bn_intp_minDT>(tempDT)){ // if better value than previous one
            bn_intp_MicrosOffset=prevMessageDate-(tempDT>>1);
            bn_intp_minDT=tempDT;
        }
    }

    prevMessageIndex=msg->payload.intp.index;
    prevMessageDate=receivedDate;

}

/* bn_intp_sync : Makes the synchronization with device at address slave.
 * Argument :
 *  slave : address of the device to synchronize (must have bn_intp installed, cf bn_intp_install).
 *  retries : number of synchronization message to send.
 * Return value :
 *  >0 if synchronization happened correctly on the master side (amount of pair of sychronization messages acked)
 *  <0 otherwise (see global_errors.h).
 */
int bn_intp_sync(bn_Address slave, int retries){
    sMsg tempMsg={{0}};
    uint8_t tempIndex=0;
    uint32_t time0=0,time1=0;
    int i=0;
    int prevAcked=0,nbAcked=0,nbSucces=0;
    int ret;

    tempMsg.header.destAddr=slave;
    tempMsg.header.type=E_INTP;
    tempMsg.header.size=sizeof(sINTP);

    for (i=0; i<retries; i++){

        tempMsg.payload.intp.index=tempIndex;   // set the index
        tempMsg.payload.intp.prevTime=time1;    // set the date of the previous sending
        time0=micros();
        tempMsg.payload.intp.time=time0;        // set the date of the current sending

        ret=bn_sendAck(&tempMsg);       // send the message

        time1=time0;

        if (ret==1 && prevAcked) nbSucces++; // synchro is successful if at least 2 messages have been acked (any retries may improve precision)
        else if (ret==1) {
            prevAcked=1;
            nbAcked++;
        }

        tempIndex++;
    }

    if (nbSucces>0) return nbSucces;
    else if (!nbAcked) return ret;      // no cak at all ? slave device is probably off
    else return -ERR_TRY_AGAIN;         // no success but some acks ? well, network very busy, you may retry and pray to have more luck.

}

