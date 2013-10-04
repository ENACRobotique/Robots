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

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

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
#error please Define The Architecture Symbol you Bloody Bastard
#endif

sMsg msgBuf[SB_INC_MSG_BUF_SIZE]={{{0}}};
int iFirst=0,iNext=0; //index of the first (oldest) message written in the buffer and index of where the next message will be written
int nbMsg=0;//nb of message available in msgBuf (enables to distinguish the case iFirst==iNext when the buffer is full form the case iFirst==iNext when the buffer is empty)

sb_Address debug_addr=ADDRX_DEBUG;


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

// TODO sb_deinit

/*
 * Handles the sending of a message over the SuperBus network
 * For user's use only, the message was previously NOT "in the network"
 */
int sb_send(sMsg *msg){
    // sets source address
    msg->header.srcAddr = (MYADDRX?:MYADDRI)?:MYADDRU;

    // sets checksum
    setSum(msg);



    // actual sending
    return sb_forward(msg, IF_LOCAL);
}


/*
 * SuperBus Routine, handles receiving messages, routing/forwarding them, putting the ones for this node in a buffer
 * Handles one message at the time, SHOULD be called in a rather fast loop
 * Non blocking function
 * Arguments : none
 * Return value :
 *  number of bytes handled
 *
 */
int sb_routine(){
    sMsg temp;
    int count=0;

#if (MYADDRX)!=0
    if (Xbee_receive(&temp)) {
        count+=sb_forward(&temp,IF_XBEE);
    }
#endif
#if (MYADDRI)!=0
    if (I2C_receive(&temp)) {
        count+=sb_forward(&temp,IF_I2C);
    }
#endif
#if (MYADDRU)!=0
    if (UART_receive(&temp)) {
        count+=sb_forward(&temp,IF_UART);
    }
#endif
    return count;
}

/*
 * pops the oldest message unread in the incoming message buffer
 * Argument :
 *  msg : pointer to the memory area where the last message will be written
 * Return value :
 *  nb of bytes written
 */
int sb_receive(sMsg *msg){

    if ( iFirst==iNext && !nbMsg) return 0;

    //pop the oldest message of incoming buffer and updates index
    memcpy(msg, &(msgBuf[iFirst]), msgBuf[iFirst].header.size + sizeof(sGenericHeader));
    iFirst=(iFirst+1)%SB_INC_MSG_BUF_SIZE;
    nbMsg--;

    return (msg->header.size + sizeof(sGenericHeader));
}


/*
 * Handles the routing of a message
 * Argument :
 *  msg : pointer to the message to send
 * Return value : structure defining the next hop to send the message to
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
 *      number of bytes written/send
 *      -1 if error
 *
 * Remark : if the message is for this node in particular, it is stored in the incoming buffer msgBuf
 */
//FIXME nexthop address
int sb_forward(sMsg *msg, E_IFACE ifFrom){
    sRouteInfo routeInfo=sb_route(msg, ifFrom);
    switch (routeInfo.ifTo){
#if MYADDRX !=0
    case IF_XBEE :
        return Xbee_send(msg, routeInfo.nextHop);
        break;
#endif
#if MYADDRI!=0
    case IF_I2C :
        return I2C_send(msg, routeInfo.nextHop);
        break;
#endif
#if MYADDRU !=0
    case IF_UART :
        return UART_send(msg, routeInfo.nextHop);
        break;
#endif
    case IF_DROP :
        return 0;
        break;
    case IF_LOCAL :

                // TODO, tell sender, message can't be sent and do not drop oldest message
        if (iFirst==iNext && nbMsg==SB_INC_MSG_BUF_SIZE) {
            iFirst=(iFirst+1)%SB_INC_MSG_BUF_SIZE; //"drop" oldest message if buffer is full
            nbMsg--;
        }

        memcpy(&msgBuf[iNext],msg, msg->header.size+sizeof(sGenericHeader));
        iNext=(iNext+1)%SB_INC_MSG_BUF_SIZE;
        nbMsg++;

        return (msg->header.size + sizeof(sGenericHeader));
        break;
    default : return -1;
    }
    return 0;
}

/* sb_printDbg : sends a fixed string in a message to the address debug_addr.
 * Arguments :
 *  str : pointer to the string to send
 * Return value :
 *  number of bytes written/send
 *  -1 if error
 *
 * Remark : this will blindly shorten the string if the latter was too big.
 */
int sb_printDbg(const char * str){
    sMsg tmp;

    tmp.header.destAddr=debug_addr;
    tmp.header.type=E_DEBUG;
    tmp.header.size=MIN(strlen(str)+1 , SB_MAX_PDU-sizeof(sGenericHeader));
    strncpy((char *)tmp.payload.data , str , SB_MAX_PDU-sizeof(sGenericHeader)-1);
    tmp.payload.debug[tmp.header.size-1]=0; //strncpy does no ensure the null-termination, so we force it

    return sb_send(&tmp);
}

/* sb_printfDbg : sends a string in a message to the address debug_addr unsing a printf-like formatting.
 * Arguments :
 *  format : see printf documentation
 * Return value :
 *  number of bytes written/send
 *  -1 if error
 *
 * Remark : this will blindly shorten the string if the latter was too big.
 */
int sb_printfDbg(char *format, ...){

    uint8_t string[SB_MAX_PDU-sizeof(sGenericHeader)];

    va_list ap;

    va_start(ap, format);
    vsnprintf((char *)string, SB_MAX_PDU-sizeof(sGenericHeader), format, ap);
    va_end(ap);

    return sb_printDbg(string);
}


/* sb_debugAddrSignal : modifies the local debug address. This should be used on receiving a signalling message of type E_DEBUG_SIGNALLING
 * Arguments :
 *  msg : pointer to the received message
 * Return value : none
 */
void sb_debugAddrSignal(sMsg * msg){

    if (msg->header.type==E_DEBUG_SIGNALLING) {
         debug_addr=msg->header.srcAddr;
    }

}

//TODO : attach mechanism to some messages
