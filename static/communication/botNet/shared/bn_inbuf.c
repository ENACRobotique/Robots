/*
 * bn_inbuf.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#include <string.h>
#include "message_header.h"

#include "global_errors.h"

#include "bn_inbuf.h"

// useful define
#ifndef MIN
#define MIN(m, n) ((m)>(n)?(n):(m))
#endif
#ifndef MAX
#define MAX(m, n) ((m)>(n)?(m):(n))
#endif

//incoming message buffer, indexes to parse it and total number of messages.
//Warning : central buffer where most of the incoming messages will be stacked
//Policy : drop oldest (except when there is an ack, the ack has always priority)
sMsgIf msgIfBuf[BN_INC_MSG_BUF_SIZE]; // DAT buffer.
int iFirst=0,iNext=0;   // index of the first (oldest) message written in the buffer and index of where the next message will be written
int nbMsg=0;            // nb of message available in msgBuf (enables to distinguish the case iFirst==iNext when the buffer is full form the case iFirst==iNext when the buffer is empty)

//architecture-specific includes
#ifdef ARCH_328P_ARDUINO
    #include "../arduino/mutex/mutex.h"
#elif defined(ARCH_X86_LINUX)
    #include <stdarg.h>
    #include "../linux/mutex/mutex.h"
#elif defined(ARCH_LPC21XX)
    #include <stdarg.h>
    #include "../lpc21xx/mutex/mutex.h"
#elif defined(ARCH_LM4FXX)
    #include "../lm4fxx/mutex/mutex.h"
#else
#error "please Define The Architecture Symbol, You Bloody Bastard"
#endif



/* bn_pushInBufLast : insert a message at the last position in the incoming message buffer
 * Argument :
 *      msg : pointer to the  message to store
 *      iFace : interface on which the message has been received
 * Return value :
 *      1
 *      <0 on error (buffer full)
 * WARNING : will drop msg if the buffer is full, unless the msg is an ack.
 */
int bn_pushInBufLast(const sMsg *msg, E_IFACE iFace){
    int iTmp;

    bn_mutexLock();
    if (nbMsg==BN_INC_MSG_BUF_SIZE) {
        // unless it is an ack, drop the message
        if (msg->header.type!=E_ACK_RESPONSE){
            bn_mutexUnlock();
            return -ERR_BN_BUFFER_FULL;
        }
        //makes some room if it is an ack
        else bn_freeInBufFirst();
    }
    iTmp=iNext;
    iNext=(iNext+1)%BN_INC_MSG_BUF_SIZE;
    nbMsg++;
    bn_mutexUnlock();

#ifdef DEBUG_PC_BUF
    {
        sMsg *msgPtr=msg;
        printf("%hx -> %hx type %u seq %u ack %u [pushBuf] iWrite %d  iFirst %d iNext %d nbMsg %d\n",msgPtr->header.srcAddr,msgPtr->header.destAddr,msgPtr->header.type,msgPtr->header.seqNum,msgPtr->header.ack,iTmp,iFirst,iNext,nbMsg);
    }
#endif

    msgIfBuf[iTmp].iFace=iFace;
    memcpy(&msgIfBuf[iTmp].msg,msg,MIN(msg->header.size+sizeof(msg->header),sizeof(*msg)));
    return 1;
}

/* bn_getAllocInBufLast() : returns a pointer to the memory area of the central buffer where it should be written and updates the pointers as if the message was written
 * Argument :
 *      none
 * Return value :
 *      pointer to the memory area where the next message/interface structure should be written
 *      NULL (buffer full)
 * WARNING : may return NULL
 * WARNING : will updates indexes if there is space, so any call to this function MUST result in a message written at the return value (unless return val==NULL)
 */
sMsgIf * bn_getAllocInBufLast(){
    sMsgIf *tmp;

    bn_mutexLock();
    if (nbMsg==BN_INC_MSG_BUF_SIZE) {
        bn_mutexUnlock();              //hum hum...
        return NULL;
    }
    tmp=&(msgIfBuf[iNext]);
    iNext=(iNext+1)%BN_INC_MSG_BUF_SIZE;
    nbMsg++;
    bn_mutexUnlock();

#ifdef DEBUG_PC_BUF
    {
        printf("? -> ? type ? seq ? ack ? [getAllocBuf] iWrite %d  iFirst %d iNext %d nbMsg %d\n",(iNext+BN_INC_MSG_BUF_SIZE-1)%BN_INC_MSG_BUF_SIZE,iFirst,iNext,nbMsg);
    }
#endif

    return tmp;
}

/* bn_popInBuf : pops the oldest message/interface structure out of the incoming message buffer
 * Argument :
 *      pstru : pointer to the memory area where the message/interface structure will be written.
 * Return value :
 *      1 on succes
 *      0 if buffer empty
 */
int bn_popInBuf(sMsgIf * pstru){
    int iTmp;

    bn_mutexLock();
    if (nbMsg==0) {
        bn_mutexUnlock();
        return 0;
    }
    //pop the oldest message of incoming buffer and updates index
    iTmp=iFirst;
    iFirst=(iFirst+1)%BN_INC_MSG_BUF_SIZE;
    nbMsg--;

#ifdef DEBUG_PC_BUF
    {
        sMsg *msgPtr=&(msgIfBuf[iFirst]);
        printf("%hx -> %hx type %u seq %u ack %u [popBuf]  iRead %d  iFirst %d iNext %d nbMsg %d\n",msgPtr->header.srcAddr,msgPtr->header.destAddr,msgPtr->header.type,msgPtr->header.seqNum,msgPtr->header.ack,iTmp,iFirst,iNext,nbMsg);
    }
#endif

    if (pstru==NULL) {
        bn_mutexUnlock();
        return -ERR_NULL_POINTER_WRITE_ATTEMPT;
    }
    memcpy(&pstru->msg, &msgIfBuf[iTmp].msg, MIN(sizeof(pstru->msg.header)+msgIfBuf[iTmp].msg.header.size,sizeof(pstru->msg)));
    pstru->iFace = msgIfBuf[iTmp].iFace;
    bn_mutexUnlock();

    return 1;
}

/* bn_getInBufFirst : returns the address of the oldest message/interface structure in of the incoming message buffer
 * Argument :
 *      none
 * Return value :
 *      pointer to the oldest message/interface
 *      NULL if buffer empty
 * WARNING : may return NULL
 * WARNING : it is mandatory to call bn_freeInBufFirst() after treatment
 */
sMsgIf *bn_getInBufFirst(){
    if (nbMsg==0) return NULL;

#ifdef DEBUG_PC_BUF
    {
        sMsg *msgPtr=&(msgIfBuf[iFirst]);
        printf("%hx -> %hx type %u seq %u ack %u [getbuf]  iRead  %d ",msgPtr->header.srcAddr,msgPtr->header.destAddr,msgPtr->header.type,msgPtr->header.seqNum,msgPtr->header.ack,iFirst);
    }
#endif
    return &(msgIfBuf[iFirst]);
}

/* bn_freeInBufFirst : Updates the indexes after a call to bn_getInBufFirst()
 *  Argument
 *      none
 * Return value :
 *      none
 * WARNING : it is mandatory to call bn_freeInBufFirst() after treatment.
 * WARNING : DO NOT call if the previous bn_getInBufFirst() returned NULL
 */
void bn_freeInBufFirst(){
    iFirst=(iFirst+1)%BN_INC_MSG_BUF_SIZE;
    nbMsg=MAX(nbMsg-1,0);

#ifdef DEBUG_PC_BUF
    {
        printf(" iFirst %d iNext %d nbMsg %d  [freeInBuf]\n",iFirst,iNext,nbMsg);
    }
#endif
}
