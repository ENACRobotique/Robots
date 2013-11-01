/*
 * lib_superbus.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#include "lib_superBus.h"
#include "lib_checksum.h"
#include "lib_sbDebug.h"
#include "Xbee4sb.h"
#include "network_cfg.h"
#include "node_cfg.h"
#include "mutex/mutex.h"
#include "timeout.h"



#include <string.h>
#include <stdlib.h>


#ifndef MIN
#define MIN(m, n) (m)>(n)?(n):(m)
#endif

#ifdef ARCH_328P_ARDUINO
    #if MYADDRU !=0
        #include "UART/lib_UART_arduino.h"
    #endif
    #if MYADDRI!=0
        #include "I2C/lib_I2C_arduino.h"
    #endif
    #if MYADDRX !=0
        #include "Xbee_API.h"
    #endif
#elif defined(ARCH_X86_LINUX)
    #include <stdarg.h>
    #if MYADDRX !=0
        #include "Xbee_API.h"
    #endif

#elif defined(ARCH_LPC21XX)
    #include <stdarg.h>

    #include "lib_I2C_lpc21xx.h"
#else
#error "please Define The Architecture Symbol, You Bloody Bastard"
#endif



//incoming message buffer, indexes to parse it and total number of messages.
//Warning : central buffer where most of the incoming messages will be stacked
sMsgIf msgIfBuf[SB_INC_MSG_BUF_SIZE]={{{{0}}}};
int iFirst=0,iNext=0; //index of the first (oldest) message written in the buffer and index of where the next message will be written
int nbMsg=0;//nb of message available in msgBuf (enables to distinguish the case iFirst==iNext when the buffer is full form the case iFirst==iNext when the buffer is empty)

//local message to "transmit" via sb_receive()
sMsg localMsg={{0}};
int localReceived=0; //indicates whether a message is available for local or not

//sb_attach structure
typedef struct sAttachdef{
    E_TYPE type;
    pfvpm func;
    struct sAttachdef *next;
} sAttach;

//sb_attach first element
sAttach *firstAttach=NULL;

//sequence number counter;
uint8_t seqNum=0;



// todo sb_deinit

/*
 * Handles the initialization of the superBus interfaces
 */
int sb_init(){

#if MYADDRU!=0
#   ifdef ARCH_X86_LINUX
    UART_init("/dev/ttyUSB0",0);
#   endif

#   ifdef ARCH_328P_ARDUINO
    UART_init(0,111111);
#   endif
#endif

#if MYADDRI!=0
#   if defined(ARCH_328P_ARDUINO) || defined(ARCH_LPC21XX)
    I2C_init(400000UL);
#   endif
#endif

#if MYADDRX!=0
    Xbee_init();
#endif
    return 0;
}


/*
 * sb_send : handles the "classic" sending of a message over the SuperBus network (no ack nor broadcast)
 * For user's use only, the message was previously NOT "in the network"
 * Arguments :
 *      msg : pointer to the message to send.
 * Return Value :
 *      see sb_forward :
 *          <0 if error
 *          >0 : nb of bytes written is correct
 *
 */
int sb_send(sMsg *msg){

    //sets ack bit
    msg->header.ack=0;

    // sending
    return sb_genericSend(msg);
}

/*
 * sb_genericSend : system use only, not for user.
 * Arguments :
 *      msg : pointer to the message to send.
 * Return Value :
 *      see sb_forward :
 *          <0 if error
 *          >0 : nb of bytes written is correct
 * Remark : will increment global seqNum. the seqNum of the send message is the value of the global var seqNum BEFORE the call to sb_genericSend.
 */
int sb_genericSend(sMsg *msg){
    // sets source address
    msg->header.srcAddr = (MYADDRX?:MYADDRI)?:MYADDRU;

    //check the size of the message
    if ( (msg->header.size + sizeof(sGenericHeader)) > SB_MAX_PDU) return -1;

    //sets type version
    msg->header.typeVersion=SB_TYPE_VERSION;

    //sets seqNum
    msg->header.seqNum=seqNum;
    seqNum+=(seqNum+1)&15;      //for a 4 bits sequence number

    // sets checksum
    setSum(msg);

    // actual sending
    return sb_forward(msg, IF_LOCAL);
}

/*
 * sb_send : handles the "acked" sending of a message over the SuperBus network (no broadcast)
 * For user's use only, the message was previously NOT "in the network"
 * Arguments :
 *      msg : pointer to the message to send.
 * Return Value :
 *      see sb_forward :
 *          -1 if error
 *          -2 if nack broken link received
 *          -3 if nack buffer full received
 *          -4 if no ack is received
 *          1  if message acked
 *  WARNING : "BLOCKING" FUNCTION (timeout SB_ACK_TIMEOUT)
 */
int sb_sendAck(sMsg *msg){
    uint32_t sw=0;  // stopwatch memory
    sMsg msgIn={{0}}; //incoming message (may be our ack)

    sb_Address tmpAddr=msg->header.destAddr;
    uint8_t tmpSeqNum=seqNum;

    // sets ack bit
    msg->header.ack=1;

    // sending
    if (sb_genericSend(msg) < 0) return -1;

    // waiting for the reply
    while ( testTimeout(SB_ACK_TIMEOUT*1000,&sw)){
        // route message (to receive the one we are waiting for)
        sb_routine();
        // if we receive a message
        if (sb_receive(&msgIn)>0){
            //if this message is not an ack response,  put it back in the incoming buffer (sent to self)
            if ( msgIn.header.type != E_ACK_RESPONSE ){
                sb_pushInBufLast(msg,IF_LOCAL);
            }
            // if this msg is not the ack expected (not the good destination or seqnum), drop it (do nothing)
            else if ( msgIn.payload.ack.addr != tmpAddr || msgIn.payload.ack.seqNum != tmpSeqNum) continue;
            else if ( msgIn.payload.ack.ans == A_ACK) return 1;
            else if ( msgIn.payload.ack.ans == A_NACK_BROKEN_LINK) return -2;
            else if ( msgIn.payload.ack.ans == A_NACK_BUFFER_FULL) return -3; //XXX behavior?
        }
    }
    return -4;
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
int sb_routine(){
    sMsgIf temp;
    sMsgIf *pTmp=NULL;
    int count=0;

#if (MYADDRX)!=0
    if ( (count=Xbee_receive(&temp.msg)) > 0 ) {
        sb_pushInBufLast(&temp.msg,IF_XBEE);
        // TODO : optimize this (sb_pushInBuf directly in Xbee_receive())
    }
    else if (count<0) return count;
    count=0;

#endif
#if (MYADDRI)!=0
    if ( (count=I2C_receive(&temp.msg))>0) {
        sb_pushInBufLast(&temp.msg,IF_I2C);
        // TODO : optimize this (sb_pushInBuf directly in I2C_receive())
    }
    else if (count<0) return count;
    count=0;
#endif
#if (MYADDRU)!=0
    if (UART_receive(&temp)>0) {
        sb_pushInBufLast(&temp.msg,IF_UART);
    }
#endif



    //handles stored messages
    if ( (pTmp=sb_getInBufFirst()) != NULL){
        //checks checksum of message before forwarding. If error, drop message and return
        if ( checkSum(&(pTmp->msg)) ){
            sb_freeInBufFirst();
            return -1;
        }
        //forward message
        count=sb_forward(&(pTmp->msg),pTmp->iFace);

        //handle the acknowledgement
        if ( pTmp->msg.header.ack == 1){
            //if the message is for us, send acknowledgement to the initial sender
            if (pTmp->msg.header.destAddr == MYADDRX || pTmp->msg.header.destAddr == MYADDRI || pTmp->msg.header.destAddr == MYADDRU ){

                temp.msg.header.destAddr=pTmp->msg.header.srcAddr;
                temp.msg.header.type=E_ACK_RESPONSE;
                temp.msg.header.size=sizeof(sAckPayload);

                temp.msg.payload.ack.ans=A_ACK;
                temp.msg.payload.ack.addr=pTmp->msg.header.destAddr;
                temp.msg.payload.ack.seqNum=pTmp->msg.header.seqNum;

                sb_send(&(temp.msg));

            }
            //else send nack on forwarding fail
            else if (count<0){
                temp.msg.header.destAddr=pTmp->msg.header.srcAddr;
                temp.msg.header.type=E_ACK_RESPONSE;
                temp.msg.header.size=sizeof(sAckPayload);

                temp.msg.payload.ack.ans=A_NACK_BROKEN_LINK;
                temp.msg.payload.ack.addr=pTmp->msg.header.destAddr;
                temp.msg.payload.ack.seqNum=pTmp->msg.header.seqNum;

                sb_send(&(temp.msg));
            }
        }

        sb_freeInBufFirst();
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
 *      -1 on error
 */
int sb_receive(sMsg *msg){
    sAttach *elem=firstAttach;

    // if no message available
    if ( !localReceived) return 0;

    // Check the type
    // is the version different ?
    if ( msg->header.typeVersion != SB_TYPE_VERSION ){
        sb_printfDbg("type version rx %u (loc. %u)",msg->header.typeVersion,SB_TYPE_VERSION);
        return -1;
    }
    // is it above the highest type this node knows ? (time to rebuild and update this node)
    if ( msg->header.type >= E_TYPE_COUNT) {
        sb_printDbg("type unknown (too big)");
        return -1;
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
sRouteInfo sb_route(sMsg *msg,E_IFACE ifFrom){
    int i=0;
    sRouteInfo routeInfo;

    // if this message if for this node (including broadcast possibilities) but not from this node XXX enable I2C broadcast rx
    if ( ifFrom!=IF_LOCAL && (
            msg->header.destAddr==MYADDRU || msg->header.destAddr==MYADDRI || (  (msg->header.destAddr & SUBNET_MASK)==(MYADDRX & SUBNET_MASK) && (msg->header.destAddr & MYADDRX & DEVICEX_MASK) ) ) ){
        routeInfo.ifTo=IF_LOCAL;
        routeInfo.nextHop=msg->header.destAddr;
        return routeInfo;
    }
    //if this message is from this node , for this node AND not a broadcast one (ie. dest address is exactly ours), treat it like an incoming message for this node
    if ( ifFrom==IF_LOCAL && (
            msg->header.destAddr==MYADDRU || msg->header.destAddr==MYADDRI || msg->header.destAddr == MYADDRX ) ){
        routeInfo.ifTo=IF_LOCAL;
        routeInfo.nextHop=msg->header.destAddr;
        return routeInfo;
    }
#if MYADDRI!=0
    // if this msg's destination is directly reachable and the message does not come from the associated interface, send directly to dest
    if ((msg->header.destAddr&SUBNET_MASK) == (MYADDRI&SUBNET_MASK) ) {
        if (ifFrom!=IF_I2C ) {
            routeInfo.ifTo=IF_I2C;
            routeInfo.nextHop=msg->header.destAddr;
            return routeInfo;
        }
        else {
            routeInfo.ifTo=IF_DROP;
            return routeInfo;
        }
    }
#endif
#if MYADDRX!=0
    if ((msg->header.destAddr & SUBNET_MASK) == (MYADDRX & SUBNET_MASK) ){
        if (ifFrom!=IF_XBEE ) {
            routeInfo.ifTo=IF_XBEE;
            routeInfo.nextHop=msg->header.destAddr;
            return routeInfo;
        }
        else {
            routeInfo.ifTo=IF_DROP;
            return routeInfo;
        }
    }
#endif

#if MYADDRU!=0
    if ((msg->header.destAddr & SUBNET_MASK) == (MYADDRU & SUBNET_MASK) ){
        if (ifFrom!=IF_UART ) {
            routeInfo.ifTo=IF_UART;
            routeInfo.nextHop=msg->header.destAddr;
            return routeInfo;
        }
        else {
            routeInfo.ifTo=IF_DROP;
            return routeInfo;
        }
    }
#endif
    // else, sweep the table until you reach the matching subnetwork or the end
    while(rTable[i].destSubnet!=(0x42&(~SUBNET_MASK))){
        if ( rTable[i].destSubnet == (msg->header.destAddr & SUBNET_MASK) ) {
            return rTable[i].nextHop;
        }
        i++;
    }
    //if you reach the end, send to default destination
    return rTable[i].nextHop;
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
int sb_forward(sMsg *msg, E_IFACE ifFrom){
    sRouteInfo routeInfo=sb_route(msg, ifFrom);
    int retVal=0,retries=0;
    switch (routeInfo.ifTo){
#if MYADDRX !=0
    case IF_XBEE :
        while (retVal<=0 && retries<SB_MAX_RETRIES){
            retVal=Xbee_send(msg, routeInfo.nextHop);
            retries++;
        }
        return retVal;
        break;
#endif
#if MYADDRI!=0
    case IF_I2C :
        while (retVal<=0 && retries<SB_MAX_RETRIES){
            retVal=I2C_send(msg, routeInfo.nextHop);
            retries++;
        }
        return retVal;
        break;
#endif
#if MYADDRU !=0
    case IF_UART :
        while (retVal<=0 && retries<SB_MAX_RETRIES){
            retVal=UART_send(msg, routeInfo.nextHop);
            retries++;
        }
        return retVal;
        break;
#endif
    case IF_DROP :
        return 0;
        break;
    case IF_LOCAL :
        // check if there are not already a message for sb_receive(). If not, give msg to sb_receive.
        // If yes, put last msg back in the central buffer (this case will happen only if the node sends a message to itself, so only the "local" message is put in the buffer)
        if (!localReceived){
            memcpy(&localMsg,msg,sizeof(sMsg));
            localReceived=1;
            return (msg->header.size + sizeof(sGenericHeader));
        }
        else {
            sb_pushInBufLast(msg,ifFrom);
        }
        break;
    default : return -1;
    }
    return 0;
}

/* sb_attach(E_TYPE type,pfvpm ptr);
 * Arguments :
 *      type : type of the message to attach to.
 *      ptr : pointeur to the function to attach.
 * Return value :
 *      0 if assignment correct
 *      -1 if wrong type
 *      -2 if type has already been assigned. (in this case, the previous attachment remains unmodified. see sb_deattach).
 *      -3 if memory allocation fails.
 *  Set an automatic call to function upon reception of a message of type "type".
 *  Warning : after a call to sb_attach, any message of type "type" received by this node WILL NOT be given to the user (won't pop with sb_receive)
 */
int sb_attach(E_TYPE type,pfvpm ptr){
    sAttach *elem=firstAttach, *prev, *new;

    //checks if the type is correct (should be within the E_TYPE enum range)
    if (type>=E_TYPE_COUNT) return -1;

    // TODO check if enough free space before allocating
    //looking for already existing occurence of this type while searching for the last element of the chain
    while ( elem!=NULL){
        if ( elem->type == type ) return -2;
        else {
            prev=elem;
            elem=elem->next;
        }
    }

    //create new entry
    if ( (new = (sAttach *)malloc(sizeof(sAttach))) == NULL ) return -3;

    //updates anchor
    if ( firstAttach==NULL) firstAttach=new;
    else prev->next=new;

    new->next=NULL;
    new->type=type;
    new->func=ptr;


    return 0;
}


/* sb_deattach(E_TYPE type);
 * Arguments :
 *      type : type of the message to remove the attachement from.
 * Return value :
 *      0 if everything went fine
 *      -1 if wrong type
 *      -2 if type not found (not previously attached, or already de-attached)
 *      -3 if memory free fails
 *  Unset an automatic call to function upon reception of a message of type "type".
 *  Warning : after a call to sb_attach, any message of type "type" received by this node WILL be given to the user (won't pop with sb_receive)
 */
int sb_deattach(E_TYPE type){
    sAttach *elem=firstAttach,*nextElem;

    //checks if the type is correct (should be within the E_TYPE enum range)
    if (type>=E_TYPE_COUNT) return -1;

    if (firstAttach==NULL) return -2;

    //looking for already existing occurence of this type
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

    return -2;
}

/* sb_insertInBuf : insert a message at the last postion in the incoming message buffer
 * Argument :
 *      msg : pointer to the  message to store
 *      iFace : interface on which the messahe has been received
 * Return value :
 *      1
 *      -1 on error (buffer full)
 * WARNING : will drop msg if the buffer is full
 */
int sb_pushInBufLast(sMsg *msg, E_IFACE iFace){
    int iTmp;

    mutexLock();
    if (nbMsg==SB_INC_MSG_BUF_SIZE) {
        mutexUnlock();
        return -1;
    }
    iTmp=iNext;
    iNext=(iNext+1)%SB_INC_MSG_BUF_SIZE;
    nbMsg++;
    mutexUnlock();

    msgIfBuf[iTmp].iFace=iFace;
    memcpy(&msgIfBuf[iTmp].msg,msg, MIN(msg->header.size+sizeof(sGenericHeader),sizeof(sMsg)));
    return 1;
}

/* sb_getAllocInBufLast() : returns a pointer to the memory area of the central buffer where it should be written and updates the pointers as if the message was written
 * Argument :
 *      none
 * Return value :
 *      pointer to the memory area where the next message/interface structure should be written
 *      NULL (buffer full)
 * WARNING : may return NULL
 * WARNING : will updates indexes if there is space, so any call to this function MUST result in a message written at the return value (unless return val==NULL)
 */
sMsgIf * sb_getAllocInBufLast(){
    sMsgIf *tmp;

    mutexLock();
    if (nbMsg==SB_INC_MSG_BUF_SIZE) {
        mutexUnlock();              //hum hum...
        return NULL;
    }
    tmp=&(msgIfBuf[iNext]);
    iNext=(iNext+1)%SB_INC_MSG_BUF_SIZE;
    nbMsg++;
    mutexUnlock();


    return tmp;
}

/* sb_popInBuf : pops the oldest message/interface structure out of the incoming message buffer
 * Argument :
 *      pstru : pointer to the memory area where the message/interface structure will be written.
 * Return value :
 *      1 on succes
 *      0 if buffer empty
 */
int sb_popInBuf(sMsgIf * pstru){
    int iTmp;

    mutexLock();
    if (nbMsg==0) {
        mutexUnlock();
        return 0;
    }
    //pop the oldest message of incoming buffer and updates index
    iTmp=iFirst;
    iFirst=(iFirst+1)%SB_INC_MSG_BUF_SIZE;
    nbMsg--;
    mutexUnlock();

    memcpy(pstru, &(msgIfBuf[iTmp]), MIN(msgIfBuf[iTmp].msg.header.size + sizeof(sGenericHeader),sizeof(sMsg)));

    return 1;
}

/* sb_getInBufFirst : returns the address of the oldest message/interface structure in of the incoming message buffer
 * Argument :
 *      none
 * Return value :
 *      pointer to the oldest message/interface
 *      NULL if buffer empty
 * WARNING : may return NULL
 * WARNING : it is mandatory to call sb_freeInBufFirst() after treatment
 */
sMsgIf *sb_getInBufFirst(){

    if (nbMsg==0) return NULL;
    return &(msgIfBuf[iFirst]);
}

/* sb_getInBufFirst : Updates the indexes after a call to sb_getInBufFirst()
 *  Argument
 *      none
 * Return value :
 *      none
 * WARNING : it is mandatory to call sb_freeInBufFirst() after treatment.
 * WARNING : DO NOT call if the previous sb_getInBufFirst() returned NULL
 */
void sb_freeInBufFirst(){

    iFirst=(iFirst+1)%SB_INC_MSG_BUF_SIZE;
    nbMsg=MAX(nbMsg-1,0);

    return;
}
