/*
 * lib_superbus.c
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
#include "bn_debug.h"
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

// useful define
#ifndef MIN
#define MIN(m, n) (m)>(n)?(n):(m)
#endif


//architecture-specific includes
#ifdef ARCH_328P_ARDUINO
    #include "../arduino/mutex/mutex.h"
#elif defined(ARCH_X86_LINUX)
    #include <stdarg.h>
    #include "../linux/mutex/mutex.h"
#elif defined(ARCH_LPC21XX)
    #include <stdarg.h>
    #include "../lpc21xx/mutex/mutex.h"
#else
#error "please Define The Architecture Symbol, You Bloody Bastard"
#endif



//incoming message buffer, indexes to parse it and total number of messages.
//Warning : central buffer where most of the incoming messages will be stacked
//Policy : drop oldest (except when there is an ack, the ack has always priority)
sMsgIf msgIfBuf[BN_INC_MSG_BUF_SIZE]={{{{0}}}}; // DAT buffer.
int iFirst=0,iNext=0;   // index of the first (oldest) message written in the buffer and index of where the next message will be written
int nbMsg=0;            // nb of message available in msgBuf (enables to distinguish the case iFirst==iNext when the buffer is full form the case iFirst==iNext when the buffer is empty)

//local message to "transmit" via bn_receive()
sMsg localMsg={{0}};
int localReceived=0; //indicates whether a message is available for local or not

//bn_attach structure
typedef struct sAttachdef{
    E_TYPE type;
    pfvpm func;
    struct sAttachdef *next;
} sAttach;

//bn_attach first element
sAttach *firstAttach=NULL;

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
#if MYADDRX!=0 || MYADDRU!=0 || MYADDRD!=0
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
    if ( (ret=UART_init(BN_UART_PATH,E_115200_8N1))<0 ) return ret;
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

        //handle the acknowledgement
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
 *      nb of bytes written
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
           localReceived--;
           return 0;
       }
       else elem=elem->next;
    }

    // if no function is attached to this type
    localReceived--;
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
    if ((msg->header.destAddr&SUBNET_MASK) == (MYADDRI&SUBNET_MASK) ) {
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
 * Handles the forwarding of a message over the SuperBus network
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

/* bn_attach(E_TYPE type,pfvpm ptr);
 * Arguments :
 *      type : type of the message to attach to.
 *      ptr : pointeur to the function to attach.
 * Return value :
 *      0 if assignment correct
 *      -1 if wrong type
 *      -2 if type has already been assigned. (in this case, the previous attachment remains unmodified. see bn_deattach).
 *      -3 if memory allocation fails.
 *  Set an automatic call to function upon reception of a message of type "type".
 *  Warning : after a call to bn_attach, any message of type "type" received by this node WILL NOT be given to the user (won't pop with bn_receive)
 */
int bn_attach(E_TYPE type,pfvpm ptr){
    sAttach *elem=firstAttach, *prev=0, *new;

    //checks if the type is correct (should be within the E_TYPE enum range)
    if (type>=E_TYPE_COUNT) return -ERR_BN_TYPE_TOO_HIGH;

    // TODO check if enough free space before allocating
    //looking for already existing occurence of this type while searching for the last element of the chain
    while ( elem!=NULL){
        if ( elem->type == type ) return -ERR_BN_TYPE_ALREADY_ATTACHED;
        else {
            prev=elem;
            elem=elem->next;
        }
    }

    //create new entry
    if ( (new = (sAttach *)malloc(sizeof(sAttach))) == NULL ) return -ERR_INSUFFICIENT_MEMORY;

    //updates anchor
    if ( firstAttach==NULL) firstAttach=new;
    else prev->next=new;

    new->next=NULL;
    new->type=type;
    new->func=ptr;

    return 0;
}


/* bn_deattach(E_TYPE type);
 * Unsets an automatic call to function upon reception of a message of type "type".
 * Arguments :
 *      type : type of the message to remove the attachement from.
 * Return value :
 *      0 if everything went fine
 *      <0 if :
 *          wrong type
 *          type not found (not previously attached, or already de-attached)
 *  Warning : after a call to bn_attach, any message of type "type" received by this node WILL be given to the user (won't pop with bn_receive)
 */
int bn_deattach(E_TYPE type){
    sAttach *elem=firstAttach,*nextElem;

    //checks if the type is correct (should be within the E_TYPE enum range)
    if (type>=E_TYPE_COUNT) return -ERR_BN_TYPE_TOO_HIGH;

    if (firstAttach==NULL) return -ERR_NOT_FOUND;

    //looking for already existing occurrence of this type
    //first element
    if (elem->type==type){
        firstAttach=elem->next;
        free(elem);
        return 0;
    }

    //rest of the chain
    while ( elem->next != NULL ){
        if ( elem->next->type == type ) {
            nextElem=elem->next->next;
            free(elem->next);
            elem->next=nextElem;
            return 0;
        }
        else elem=elem->next;
    }

    return -ERR_NOT_FOUND;
}

/* bn_insertInBuf : insert a message at the last postion in the incoming message buffer
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

    mutexLock();
    if (nbMsg==BN_INC_MSG_BUF_SIZE) {
        // unless it is an ack, drop the message
        if (msg->header.type!=E_ACK_RESPONSE){
            mutexUnlock();
            return -ERR_BN_BUFFER_FULL;
        }
        //makes some room if it is an ack
        else bn_freeInBufFirst();
    }
    iTmp=iNext;
    iNext=(iNext+1)%BN_INC_MSG_BUF_SIZE;
    nbMsg++;
    mutexUnlock();

#ifdef DEBUG_PC_BUF
    {
        sMsg *msgPtr=msg;
        printf("%hx -> %hx type %u seq %u ack %u [pushBuf] iWrite %d  iFirst %d iNext %d nbMsg %d\n",msgPtr->header.srcAddr,msgPtr->header.destAddr,msgPtr->header.type,msgPtr->header.seqNum,msgPtr->header.ack,iTmp,iFirst,iNext,nbMsg);
    }
#endif

    msgIfBuf[iTmp].iFace=iFace;
    memcpy(&(msgIfBuf[iTmp].msg),msg, MIN(msg->header.size+sizeof(sGenericHeader),sizeof(sMsg)));
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

    mutexLock();
    if (nbMsg==BN_INC_MSG_BUF_SIZE) {
        mutexUnlock();              //hum hum...
        return NULL;
    }
    tmp=&(msgIfBuf[iNext]);
    iNext=(iNext+1)%BN_INC_MSG_BUF_SIZE;
    nbMsg++;
    mutexUnlock();

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

    mutexLock();
    if (nbMsg==0) {
        mutexUnlock();
        return 0;
    }
    //pop the oldest message of incoming buffer and updates index
    iTmp=iFirst;
    iFirst=(iFirst+1)%BN_INC_MSG_BUF_SIZE;
    nbMsg--;
    mutexUnlock();

#ifdef DEBUG_PC_BUF
    {
        sMsg *msgPtr=&(msgIfBuf[iFirst]);
        printf("%hx -> %hx type %u seq %u ack %u [popBuf]  iRead %d  iFirst %d iNext %d nbMsg %d\n",msgPtr->header.srcAddr,msgPtr->header.destAddr,msgPtr->header.type,msgPtr->header.seqNum,msgPtr->header.ack,iTmp,iFirst,iNext,nbMsg);
    }
#endif

    if (pstru==NULL) return -ERR_NULL_POINTER_WRITE_ATTEMPT;
    memcpy(pstru, &(msgIfBuf[iTmp]), MIN(msgIfBuf[iTmp].msg.header.size + sizeof(sGenericHeader),sizeof(sMsg)));

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

/* bn_getInBufFirst : Updates the indexes after a call to bn_getInBufFirst()
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
    return;
}

