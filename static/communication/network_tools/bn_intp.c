/*
 * bn_ntpi.c : botnet's Network Time Protocol Inverted (master-slave instead of client-server in plain NTP)
 *
 *  Created on: 2 mai 2014
 *      Author: quentin
 */

#include "bn_intp.h"

#include "../../tools/libraries/Timeout/timeout.h"

#include "../../global_errors.h"

#ifdef ARCH_328P_ARDUINO
#include "Arduino.h"
#elif defined(ARCH_X86_LINUX)
#include "../../../core/linux/libraries/Millis/millis.h"
#elif defined(ARCH_LPC21XX)
#include <sys_time.h>
#else
#error "no ARCH_XXX symbol defined"
#endif

/*
 *    master (ma_micros())       slave (sl_micros())
 *
 *    intp.prevTime | --__          |
 *                  |     --__      |
 *                  |         -->   | prevMessageDate
 *                  |          __-- |
 *                  |     __ACK     |
 *                  |  <--          |
 *    intp.time     | --__          |
 *                  |     --__      |
 *                  |         -->   | receivedDate
 *
 *    we are in bn_intp_msgHandle(), we just wrote receivedDate,
 *    we have:
 *          ma_micros() == sl_micros() - µsOffset
 *    which may be written as: (at prevMessageDate time, assuming same time for upload and download)
 *          (intp.time + intp.prevTime)/2 == prevMessageDate - µsOffset
 *    or:   µsOffset = prevMessageDate - (intp.time + intp.prevTime)/2
 *    or:   µsOffset = prevMessageDate - intp.time + (intp.time - intp.prevTime)/2
 *
 */

// XXX: idea: comparing receivedDate - prevMessageDate and intp.time - intp.prevTime could give an idea of the robustness of the synchronization
// XXX: alternate idea: remove prevTime field from E_INTP messages and synchronize using equation: intp.time == (receivedDate + prevMessageDate)/2 - µsOffset
//          bn_intp_msgHandle() is called after completion of the bn_send() for the ACK, might be a better idea to use the current equation with prevTime

uint32_t bn_intp_MicrosOffset=0;
uint32_t bn_intp_minDT=0;        // T_i - T_i-1  associated to the above value, 0 if no synchro has been measured.


/* bn_intpMsgHandle : handles synchronization messages from master.
 * MUST be bn_attached on every slave to messages of type E_INTP (done in bn_intp_install)
 *
 */
void bn_intp_msgHandle(sMsg *msg){
    static uint8_t prevMessageIndex=0;
    static uint32_t prevMessageDate=0;
    uint32_t receivedDate=micros();
    uint32_t tempDT;

    if(msg->header.type == E_INTP){
        if (prevMessageIndex==(msg->payload.intp.index-1) && msg->payload.intp.time && msg->payload.intp.prevTime){
            tempDT=msg->payload.intp.time-msg->payload.intp.prevTime;

            if (bn_intp_minDT==0 || bn_intp_minDT>tempDT){ // if better value than previous one
                bn_intp_MicrosOffset=prevMessageDate-msg->payload.intp.time+(tempDT>>1);
                bn_intp_minDT=(tempDT?tempDT:1);

                printf("sync: off%u dt%u\n", bn_intp_MicrosOffset, bn_intp_minDT);
            }
        }

        prevMessageIndex=msg->payload.intp.index;
        prevMessageDate=receivedDate;
    }
}

/* bn_intp_sync : Makes the synchronization with device at address slave.
 * Argument :
 *  slave : address of the device to synchronize (must have bn_intp installed, cf bn_intp_install).
 *  retries : number of synchronization message to send.
 * Return value :
 *  >0 if synchronization happened correctly on the master side (amount of pair of sychronization messages acked)
 *  <0 otherwise (see global_errors.h), or details in the code here.
 */
int bn_intp_sync(bn_Address slave, int retries){
    sMsg tempMsg={{0}}; // message on stack, initialized to 0s
    int i=0;
    int prevAcked=0,nbAcked=0,nbSuccess=0;
    int ret=0;

    tempMsg.header.destAddr=slave;
    tempMsg.header.type=E_INTP;
    tempMsg.header.size=sizeof(tempMsg.payload.intp);

    for (i=0; i<retries; i++){
        tempMsg.payload.intp.index=i;   // set the index
        tempMsg.payload.intp.prevTime=tempMsg.payload.intp.time;    // set the date of the previous sending (if it is 0, the message will be considered as the first one)
        tempMsg.payload.intp.time=micros();        // set the date of the current sending

        ret=bn_sendAck(&tempMsg);       // send the message

        if (ret==1 && prevAcked) nbSuccess++; // synchro is successful if at least 2 successive messages have been acked (any retries may improve precision)
        else if (ret==1) {
            prevAcked=1;
            nbAcked++;
        }
    }

    if (nbSuccess>0) return nbSuccess;
    else if (!nbAcked) return ret;      // no ak at all ? slave device is probably off, return last error code
    else return -ERR_TRY_AGAIN;         // no success but some acks ? well, network very busy, you may retry and pray to have more luck.
}
