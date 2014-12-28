/*
 * botNet_core.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

// config files
#include "network_cfg.h"
#include "node_cfg.h"

// other required libraries
#include "../../../tools/libraries/Timeout/timeout.h"
#include "../../../global_errors.h"

// botNet specific libraries
#include "botNet_core.h"
#include "bn_checksum.h"
#if (defined(DEBUG_PC_ACK) || defined(DEBUG_PC_RT) || defined(DEBUG_PC_BUF))
#include "../../network_tools/bn_debug.h"
#endif

#if MYADDRX !=0
#   include "Xbee4bn.h"
#endif
#if MYADDRI !=0
#   include "I2C4bn.h"
#endif
#if MYADDRU !=0
#   include "UART4bn.h"
#endif
#if MYADDRD !=0
#   include "UDP4bn.h"
#endif

// standard libraries
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//local message to "transmit" via bn_receive()
sMsg localMsg={{0}};
int localReceived=0; //indicates whether a message is available for local or not

//sequence number counter;
uint8_t seqNum=0;



// todo bn_deinit

/*
 * Handles the initialization of the superBus interfaces
 * return value :
 *  0 if OK
 *  <0 if error
 */
int bn_init(){
#if MYADDRX!=0 || (MYADDRU!=0 && (defined(ARCH_X86_LINUX) || defined(ARCH_328P_ARDUINO))) || MYADDRD!=0
    int ret=0;
#endif

#if MYADDRX!=0
    if ( (ret=Xbee_init())<0 ) return ret;
    if ( (ret=Xbee_setup())<0 ) return ret;
#endif

#if MYADDRI!=0
#   if defined(ARCH_328P_ARDUINO) || defined(ARCH_LPC21XX)
    I2C_init(400000UL);
#   endif
#endif

#if MYADDRU!=0
#   ifdef ARCH_X86_LINUX
    if ( (ret=UART_init(BN_UART_PATH,E_115200_8N1|E_FRAMEBASED))<0 ) return ret;
#   endif

#   ifdef ARCH_328P_ARDUINO
    if ( (ret=UART_init(NULL,115200))<0 ) return ret;
#   endif
#endif

#if MYADDRD!=0
    if((ret=UDP_init()) < 0) return ret;
#endif
    return 0;
}

/*
 * TODO
 */
int bn_sendRetry(sMsg *msg, int retries){
    int ret;

    do{
        ret = bn_sendAck(msg);
    }while(ret<=0 && --retries>0);

    return ret;
}

/*
 * bn_send : handles the "classic" sending of a message over the SuperBus network (no ack nor broadcast)
 * For user's use only, the message was previously NOT "in the network"
 * Arguments :
 *      msg : pointer to the message to send.
 * Return Value :
 *      see bn_forward :
 *          >0 : nb of bytes written is correct
 *          <0 if error
 * /!\ BEFORE calling bn_send one must fill (in msg):
 *      * header.destAddr
 *      * header.size
 *      * header.type
 *      * payload
 */
int bn_send(sMsg *msg){
    //sets ack bit
    msg->header.ack=0;

    // sending
    return bn_genericSend(msg);
}

/*
 * bn_genericSend : system use only, not for user.
 * Arguments :
 *      msg : pointer to the message to send.
 * Return Value :
 *      see bn_forward :
 *          <0 if error
 *          >0 : nb of bytes written is correct
 * Remark : will increment global seqNum. the seqNum of the send message is the value of the global var seqNum BEFORE the call to bn_genericSend.
 */
int bn_genericSend(sMsg *msg){
    // sets source address
    msg->header.srcAddr = ((MYADDRX?:MYADDRI)?:MYADDRU)?:MYADDRD;

    //check the size of the message
    if ( (msg->header.size + sizeof(sGenericHeader)) > BN_MAX_PDU) return -ERR_BN_OVERSIZE;

    //sets type version
    msg->header.typeVersion=BN_TYPE_VERSION;

    //sets seqNum
    msg->header.seqNum=seqNum;
    //updates seqnum for next message
    seqNum=(seqNum+1)&15;      //for a 4 bits sequence number

    // sets checksum
    setSum(msg);

    // actual sending
    return bn_forward(msg, IF_LOCAL);
}

/*
 * bn_sendAck : handles the "acked" sending of a message over the SuperBus network (no broadcast)
 * For user's use only, the message was previously NOT "in the network"
 * Arguments :
 *      msg : pointer to the message to send.
 * Return Value :
 *      1  if message acked
 *      <0 if error
 *
 * /!\ BEFORE calling bn_send one must fill (in msg):
 *      * header.destAddr
 *      * header.size
 *      * header.type
 *      * payload
 *  WARNING : "BLOCKING" FUNCTION (timeout BN_ACK_TIMEOUT)
 */
int bn_sendAck(sMsg *msg){
    uint32_t sw=0;  // stopwatch memory
    sMsg msgIn={{0}}; //incoming message (may be our ack)
    int ret=0;

    bn_Address tmpAddr=msg->header.destAddr;
    uint8_t tmpSeqNum=seqNum;

    // sets ack bit
    msg->header.ack=1;

    // sending
    if ( (ret=bn_genericSend(msg)) < 0) return ret;

    // waiting for the reply
    while ( testTimeout(BN_ACK_TIMEOUT*1000UL,&sw)){
        // if we receive a message
        if (bn_receive(&msgIn)>0){
#ifdef DEBUG_PC_ACK
            {
            sMsg *msgPtr=&msgIn;
            printf("%hx -> %hx type %u seq %u ack %u [sndack received], ack addr %hx ans %d seq %d\n",msgPtr->header.srcAddr,msgPtr->header.destAddr,msgPtr->header.type,msgPtr->header.seqNum,msgPtr->header.ack,msgPtr->payload.ack.addr,msgPtr->payload.ack.ans,msgPtr->payload.ack.seqNum);
            }
#endif
            //if this message is not an ack response,  put it back in the incoming buffer (sent to self)
            if ( msgIn.header.type != E_ACK_RESPONSE ){
                bn_pushInBufLast(&msgIn,IF_LOCAL);
            }
            // if this msg is not the ack expected (not the good destination or seqnum), drop it (do nothing)
            else if ( msgIn.payload.ack.addr != tmpAddr || msgIn.payload.ack.seqNum != tmpSeqNum) continue;
            else if ( msgIn.payload.ack.ans == A_ACK) return 1;
            else if ( msgIn.payload.ack.ans == A_NACK_BROKEN_LINK) return -ERR_BN_NACK_BROKEN_LINK;
            else if ( msgIn.payload.ack.ans == A_NACK_BUFFER_FULL) return -ERR_BN_NACK_FULL_BUFFER; //XXX behavior?
        }
    }
    return -ERR_BN_ACK_TIMEOUT;
}

/*
 * SuperBus Routine, handles receiving messages, routing/forwarding them, putting them in a buffer
 * receives several messages at a time
 * Handles one message at the time, SHOULD be called in a rather fast loop
 * Non blocking function
 * Arguments : none
 * Return value :
 *  >=0 : number of bytes handled
 *  <0 : error
 *
 */
int bn_routine(){
    sMsgIf temp={{{0}}};
    sMsgIf *pTmp=NULL;
    int count=0,ret=0; //count : indicator, used for debug purposes

#if (MYADDRX)!=0
    if ( (ret=Xbee_receive(&temp.msg)) > 0 ) {
        bn_pushInBufLast(&temp.msg,IF_XBEE);
        // TODO : optimize this (bn_pushInBuf directly in Xbee_receive())
    }
    else if (ret<0) return ret;
    count+=ret;
#endif

#if (MYADDRI)!=0
    if ( (ret=I2C_receive(&temp.msg)) > 0 ) {
        bn_pushInBufLast(&temp.msg,IF_I2C);
        // TODO : optimize this (bn_pushInBuf directly in I2C_receive())
    }
    else if (ret<0) return ret;
    count+=ret;
#endif

#if (MYADDRU)!=0
    if ( (ret=UART_receive(&temp.msg)) > 0 ) {
        bn_pushInBufLast(&temp.msg,IF_UART);
    }
    else if (ret<0) return ret;
    count+=ret;
#endif

#if MYADDRD!=0
    if ( (ret=UDP_receive(&temp.msg)) > 0 ) {
        bn_pushInBufLast(&temp.msg,IF_UDP);
    }
    else if (ret<0) return ret;
    count+=ret;
#endif

    //handles stored messages
    if ( (pTmp=bn_getInBufFirst()) != NULL){
        //checks checksum of message before forwarding. If error, drop message and return
        if ( checkSum(&(pTmp->msg))==-ERR_BN_CSUM ){
            bn_freeInBufFirst();
            return -ERR_BN_CSUM;
        }

#ifdef DEBUG_PC_RT
    {
        sMsg *msgPtr=&(pTmp->msg);
        int f;
        printf("%hx -> %hx type %u seq %u ack %u [routine] pload (%u bytes) : ",msgPtr->header.srcAddr,msgPtr->header.destAddr,msgPtr->header.type,msgPtr->header.seqNum,msgPtr->header.ack,msgPtr->header.size);
        for(f=0;f<msgPtr->header.size;f++){
            printf("%hx ",msgPtr->payload.raw[f]);
        }
        printf("\n");
    }
#endif

        //handle the traceroute (sends reply)
        if (pTmp->msg.header.type == E_TRACEROUTE_REQUEST){
            temp.msg.header.destAddr=pTmp->msg.header.srcAddr;
            temp.msg.header.type=E_TRACEROUTE_RESPONSE;
            temp.msg.header.size=0;
            bn_send(&(temp.msg));
            //destroy the incoming message if we were the destination
            if (
                pTmp->msg.header.destAddr == MYADDRX ||
                pTmp->msg.header.destAddr == MYADDRI ||
                pTmp->msg.header.destAddr == MYADDRU ||
                pTmp->msg.header.destAddr == MYADDRD
            ){
                bn_freeInBufFirst();
                return count;
            }
        }

        //forward message
        count=bn_forward(&(pTmp->msg),pTmp->iFace);

        //handle the acknowledgment
        if ( pTmp->msg.header.ack == 1){
            //if the message is for us, send acknowledgment to the initial sender
            if (
                pTmp->msg.header.destAddr == MYADDRX ||
                pTmp->msg.header.destAddr == MYADDRI ||
                pTmp->msg.header.destAddr == MYADDRU ||
                pTmp->msg.header.destAddr == MYADDRD
            ){
                temp.msg.header.destAddr=pTmp->msg.header.srcAddr;
                temp.msg.header.type=E_ACK_RESPONSE;
                temp.msg.header.size=sizeof(sAckPayload);

                temp.msg.payload.ack.ans=A_ACK;
                temp.msg.payload.ack.addr=pTmp->msg.header.destAddr;
                temp.msg.payload.ack.seqNum=pTmp->msg.header.seqNum;

                bn_send(&(temp.msg));
            }
            //else send nack on forwarding fail
            else if (count<0){
                temp.msg.header.destAddr=pTmp->msg.header.srcAddr;
                temp.msg.header.type=E_ACK_RESPONSE;
                temp.msg.header.size=sizeof(sAckPayload);

                temp.msg.payload.ack.ans=A_NACK_BROKEN_LINK;
                temp.msg.payload.ack.addr=pTmp->msg.header.destAddr;
                temp.msg.payload.ack.seqNum=pTmp->msg.header.seqNum;

                bn_send(&(temp.msg));
            }
        }

        bn_freeInBufFirst();
    }

    return count;
}

/*
 * pops the oldest message unread in the incoming message buffer
 * Argument :
 *      msg : pointer to the memory area where the last message will be written
 * Return value :
 *      nb of bytes written to msg if a message has been received
 *      0 if nothing is available
 *      <0 on error
 */
int bn_receive(sMsg *msg){
    sAttach *elem=firstAttach;
    int ret=0;

    // run bn_routine (to receive messages)
    if ((ret=bn_routine())<0) return ret;

    // if no message available
    if ( !localReceived) return 0;

    // Check the type
    // is the version different ?
    if ( localMsg.header.typeVersion != BN_TYPE_VERSION ){
        return -ERR_BN_TYPE_VERSION;
    }
    // is it above the highest type this node knows ? (time to rebuild and update this node)
    if ( localMsg.header.type >= E_TYPE_COUNT) {
        return -ERR_BN_TYPE_TOO_HIGH;
    }

    // checks if there are any functions attached to the type of the incoming message.
    // if so, run it silently an removes message.
    while ( elem!=NULL ){
       if ( elem->type == localMsg.header.type ) {
           //call attached function
           elem->func(&localMsg);
           localReceived=0;
           return 0;
       }
       else elem=elem->next;
    }

    // if no function is attached to this type
    localReceived=0;
    if (msg==NULL) return -ERR_NULL_POINTER_WRITE_ATTEMPT;
    memcpy(msg,&localMsg,sizeof(sMsg));
    return (msg->header.size + sizeof(sGenericHeader));
}


/*
 * Handles the routing of a message
 * Argument :
 *      msg : pointer to the message to send
 * Return value :
 *      structure defining the next hop to send the message to
 *
 * Remark : routing tables are defined in network_cfg.h & network_cfg.cpp
 */
void bn_route(const sMsg *msg,E_IFACE ifFrom, sRouteInfo *routeInfo){
    int i=0;

    // if this message is for this node (including broadcast possibilities) but not from this node XXX enable I2C broadcast rx
    if ( ifFrom!=IF_LOCAL &&
        (
            ( (msg->header.destAddr & SUBNET_MASK)==(MYADDRX & SUBNET_MASK) && (msg->header.destAddr & MYADDRX & DEVICEX_MASK) ) ||
            msg->header.destAddr==MYADDRI ||
            msg->header.destAddr==MYADDRU ||
            msg->header.destAddr==MYADDRD
        )
    ){// xxx improve test
        routeInfo->ifTo=IF_LOCAL;
        routeInfo->nextHop=msg->header.destAddr;
        return;
    }
    //if this message is from this node , for this node AND not a broadcast one (ie. dest address is exactly ours), treat it like an incoming message for this node
    if ( ifFrom==IF_LOCAL &&
        (
            msg->header.destAddr==MYADDRX ||
            msg->header.destAddr==MYADDRI ||
            msg->header.destAddr==MYADDRU ||
            msg->header.destAddr==MYADDRD
        )
    ){
        routeInfo->ifTo=IF_LOCAL;
        routeInfo->nextHop=msg->header.destAddr;
        return;
    }

    // if this msg's destination is directly reachable and the message does not come from the associated interface, send directly to dest
#if MYADDRX!=0
    if ((msg->header.destAddr & SUBNET_MASK) == (MYADDRX & SUBNET_MASK) ){
        if (ifFrom!=IF_XBEE ) {
            routeInfo->ifTo=IF_XBEE;
            routeInfo->nextHop=msg->header.destAddr;
            return;
        }
        else {
            routeInfo->ifTo=IF_DROP;
            return;
        }
    }
#endif

#if MYADDRI!=0
    if ((msg->header.destAddr & SUBNET_MASK) == (MYADDRI & SUBNET_MASK) ) {
        if (ifFrom!=IF_I2C ) {
            routeInfo->ifTo=IF_I2C;
            routeInfo->nextHop=msg->header.destAddr;
            return;
        }
        else { //do no resent on I2C a message received on I2C
            routeInfo->ifTo=IF_DROP;
            return;
        }
    }
#endif

#if MYADDRU!=0
    if ((msg->header.destAddr & SUBNET_MASK) == (MYADDRU & SUBNET_MASK) ){
        if (ifFrom!=IF_UART ) {
            routeInfo->ifTo=IF_UART;
            routeInfo->nextHop=msg->header.destAddr;
            return;
        }
        else {
            routeInfo->ifTo=IF_DROP;
            return;
        }
    }
#endif

#if MYADDRD!=0
    if ((msg->header.destAddr & SUBNET_MASK) == (MYADDRD & SUBNET_MASK) ){
        if (ifFrom!=IF_UDP) {
            routeInfo->ifTo=IF_UDP;
            routeInfo->nextHop=msg->header.destAddr;
            return;
        }
        else {
            routeInfo->ifTo=IF_DROP;
            return;
        }
    }
#endif

    // else, sweep the table until you reach the matching subnetwork or the end
    while(rTable[i].destSubnet!=(0x42&(~SUBNET_MASK))){
        if ( rTable[i].destSubnet == (msg->header.destAddr & SUBNET_MASK) ) {
            break;
        }
        i++;
    }
    //if you reach the end, send to default destination
    *routeInfo = rTable[i].nextHop;
}




/*
 * Handles the forwarding of a message over the botNet network
 * Arguments :
 *     msg : pointer to the message to send
 *     ifFrom : interface (physical or virtual) on which the message has been received
 * Return value :
 *     >0 : number of bytes written/send
 *     <0 if error
 *
 * Remark : if the message is for this node in particular, it is stored in the incoming buffer msgBuf
 */
int bn_forward(const sMsg *msg, E_IFACE ifFrom){
    sRouteInfo routeInfo;
    int retVal=0,retries=0;

    bn_route(msg, ifFrom, &routeInfo);

#ifdef DEBUG_PC_RT
        {
        sMsg *msgPtr=msg;
        printf("%hx -> %hx type %u seq %u ack %u [forward] ifto %d nexthop %hx\n",msgPtr->header.srcAddr,msgPtr->header.destAddr,msgPtr->header.type,msgPtr->header.seqNum,msgPtr->header.ack,routeInfo.ifTo,routeInfo.nextHop);
        }
#endif

    switch (routeInfo.ifTo){
#if MYADDRX !=0
    case IF_XBEE :
        while (retVal<=0 && retries<BN_MAX_RETRIES){
            retVal=Xbee_send(msg, routeInfo.nextHop);
            retries++; //FIXME : handling duplicate receive (ie check last seqnum in bn_routine)
        }
        return retVal;
        break;
#endif
#if MYADDRI!=0
    case IF_I2C :
        while (retVal<=0 && retries<BN_MAX_RETRIES){
            retVal=I2C_send(msg, routeInfo.nextHop);
            retries++;
        }
        return retVal;
        break;
#endif
#if MYADDRU !=0
    case IF_UART :
        while (retVal<=0 && retries<BN_MAX_RETRIES){
            retVal=UART_send(msg);
            retries++;
        }
        return retVal;
        break;
#endif
#if MYADDRD !=0
    case IF_UDP :
        while (retVal<=0 && retries<BN_MAX_RETRIES){
            retVal=UDP_send(msg, routeInfo.nextHop);
            retries++;
        }
        return retVal;
        break;
#endif
    case IF_DROP :
        return 0;
        break;
    case IF_LOCAL :
        // check if there is not already a message for bn_receive(). If not, give msg to bn_receive.
        if (!localReceived){
            memcpy(&localMsg,msg,sizeof(sMsg));
            localReceived=1;
            return (msg->header.size + sizeof(sGenericHeader));
        }
        // If yes, put last msg back in the central buffer (this case should normally happen only if the node sends a message to itself, so only the "local" message is put in the buffer)
        else {
            bn_pushInBufLast(msg,ifFrom);
        }
        break;
    default : return -ERR_BN_NO_SUCH_INTERFACE;
    }
    return 0;
}
