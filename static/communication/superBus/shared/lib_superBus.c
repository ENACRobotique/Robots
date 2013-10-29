/*
 * lib_superbus.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#include "lib_superBus.h"
#include "lib_checksum.h"
#include "Xbee4sb.h"
#include "network_cfg.h"
#include "node_cfg.h"
#include "mutex/mutex.h"

E_TYPE toto;

#include <string.h>
#include <stdlib.h>


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

//local message to transmit via sb_receive()
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



/*
 * Handles the initialization of the superBus interfaces
 */
int sb_init(){
    // FIXME use macros defined in params.h

#if MYADDRU!=0
#   ifdef ARCH_X86_LINUX
    UART_initSerial("/dev/ttyUSB0");
#   endif

#   ifdef ARCH_328P_ARDUINO
    UART_init(111111);
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

// todo sb_deinit

/*
 * sb_send : handles the sending of a message over the SuperBus network
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
    // sets source address
    msg->header.srcAddr = (MYADDRX?:MYADDRI)?:MYADDRU;

    if ( (msg->header.size + sizeof(sGenericHeader)) > SB_MAX_PDU) return -1;

    // sets checksum
    setSum(msg);

    // actual sending
    return sb_forward(msg, IF_LOCAL);
}


/*
 * SuperBus Routine, handles receiving messages, routing/forwarding them, putting them in a buffer
 * receives several messages at a time
 * Handles one message at the time, SHOULD be called in a rather fast loop
 * Non blocking function
 * Arguments : none
 * Return value :
 *  number of bytes handled
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
//        count+=sb_forward(&temp.msg,IF_XBEE);
    }
    else if (count<0) return count;
    count=0;

#endif
#if (MYADDRI)!=0
    if (I2C_receive(&temp.msg)>0) {
        sb_pushInBufLast(&temp.msg,IF_I2C);
        // TODO : optimize this (sb_pushInBuf directly in I2C_receive())
//        count+=sb_forward(&temp.msg,IF_I2C);
    }
#endif
#if (MYADDRU)!=0
    if (UART_receive(&temp)>0) {
        sb_pushInBufLast(&temp.msg,IF_UART);
//        count+=sb_forward(&temp,IF_UART);
    }
#endif



    //handles stored messages
    if ( (pTmp=sb_getInBufFirst()) != NULL){
        //checks checksum of message before forwarding
        if ( checksumHead(&pTmp->msg.header)==0 || checksumPload(&pTmp->msg)==0){
            sb_freeInBufFirst();
            return -1;
        }
        count+=sb_forward(&(pTmp->msg),pTmp->iFace);
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
 */
int sb_receive(sMsg *msg){
    sAttach *elem=firstAttach;

    //if no message available
    if ( !localReceived) return 0;

    //checks if there are any functions attached to the type of the incoming message.
    //if so, run it silently an removes message.
    while ( elem!=NULL ){
       if ( elem->type == localMsg.header.type ) {
           //call attached function
           elem->func(&localMsg);
           localReceived--;
           return 0;
       }
       else elem=elem->next;
    }

    //if no function is attached to this type
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

    // if this message if for us
    if ( ifFrom!=IF_LOCAL && (
            msg->header.destAddr==MYADDRU || msg->header.destAddr==MYADDRI || (  (msg->header.destAddr & SUBNET_MASK)==(MYADDRX & SUBNET_MASK) && (msg->header.destAddr & MYADDRX & DEVICEX_MASK) ) ) ){
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

        memcpy(&localMsg,msg,sizeof(sMsg));
        localReceived=1;
        return (msg->header.size + sizeof(sGenericHeader));
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

    //looking for already existing occurence of this type
    //first element
    if (elem->type==type){
        firstAttach=elem->next;
        free(elem);
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

    msgIfBuf[iNext].iFace=iFace;
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
