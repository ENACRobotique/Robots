/*
 * lib_superbus.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */



#include "lib_superBus.h"

#ifdef ARCH_328P_ARDUINO
    #include "lib_Xbee_arduino.h"
#elif defined(ARCH_X88_WILLNOTWORKBECAUSECPP)
    #include "lib_Xbee_x86.h"
#else
pleaseDefineTheArchitectureSymbol youBloodyBastard;
#endif

sMsg msgBuf[SB_INC_BUF_SIZE];
int iFirst=0,iLast=0; //index of the first and the last message written in the buffer


/*
 * Handles the sending of a message over the SuperBus network
 *
 * Remark : if the destination is the node himself, the message is put in the msgBuf buffer, and the indexes are updated accordingly
 *
 */
int sb_send(sMsg *msg){
    int i;
    E_IFACE interface=IF_UNKNOWN;



    return 0;
}


/*
 * SuperBus Routine, handles receiving messages, routing/forwarding them, putting the ones for this node in a buffer
 * Handles one message at the time, SHOULD be called in a rather fast loop
 * Non blocking function
 * Arguments : none
 * Return value :
 *  number of message available for this node
 *
 */
int sb_routine(){
    sMsg temp;

    //if Xbee receive
    if (Xbee_receive(&temp)){
        sb_send(&tmp);
    }
    //and if I2C receive TODO : "or" or "and"?

    //routing
        //if for self, put in buffer

    return 0;
}

/*
 * pop the oldest message unread in the incoming message buffer
 * Argument :
 *  msg : pointer to the memory area where the last message will be written
 * Return value :
 *  nb of bytes written
 */
int sb_receive(sMsg *msg){
    //pop oldest message put in incoming buffer


    return 0;
}


/*
 * Handles the routing of a message
 * Argument :
 *  msg : pointer to the message to send
 * Return value : interface identifier, IF_UNKNOWN if unknown
 *
 */
E_IFACE sb_route(sMsg *msg,E_IFACE from){
    int i=0;
    do {

        }while (rTable[i].destSubnet!=0);
    return IF_UNKNOWN;
}



















