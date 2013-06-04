/*
 * lib_superbus.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */



#include "lib_superBus.h"
#include "lib_checksum.h"
#include "network_cfg.h"
#include "params.h"

#include <string.h>


#ifdef ARCH_328P_ARDUINO
    #include "lib_Xbee_arduino.h"
#elif defined(ARCH_X86_WILLNOTWORKBECAUSECPP)
    #include "lib_Xbee_x86.h"
#else
pleaseDefineTheArchitectureSymbol youBloodyBastard;
#endif

sMsg msgBuf[SB_INC_MSG_BUF_SIZE];
int iFirst=0,iLast=0; //index of the first and the last message written in the buffer


/*
 * Handles the sending of a message over the SuperBus network
 * For user's use only, the message was previously NOT in the network
 */
int sb_send(sMsg *msg){
	//checksum and stuff
	setSum(msg);

	//actual sending
	sb_forward(msg,IF_LOCAL);

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
    if (Xbee_receive(&temp)) {
    	sb_forward(&temp,IF_XBEE);
    }
    //TODO if (i2c_receive...)


    return (iFirst-iLast)%SB_INC_MSG_BUF_SIZE;
}

/*
 * pops the oldest message unread in the incoming message buffer
 * Argument :
 *  msg : pointer to the memory area where the last message will be written
 * Return value :
 *  nb of bytes written
 */
int sb_receive(sMsg *msg){

	if (iFirst==iLast) return 0;

    //pop the oldest message of incoming buffer and updates index
	memcpy(msg, &(msgBuf[iFirst]), msgBuf[iFirst].header.size + sizeof(sGenericHeader));
	iFirst=(iFirst+1)%SB_INC_MSG_BUF_SIZE;

    return (msg->header.size + sizeof(sGenericHeader));
}


/*
 * Handles the routing of a message
 * Argument :
 *  msg : pointer to the message to send
 * Return value : subnet to send the message on
 */
E_IFACE sb_route(sMsg *msg,E_IFACE ifFrom){
	int i=0;

	// if this message if for us
	if (msg->header.destAddr==MYADDRI || (  (msg->header.destAddr & SUBNET_MASK)==(MYADDRX & SUBNET_MASK) && (msg->header.destAddr & MYADDRX & DEVICEX_MASK) ) ) return IF_LOCAL;


	// if this msg's destination is directly reachable and the message does not come from the associated interface, send directly to dest
	if ((msg->header.destAddr&SUBNET_MASK) == (MYADDRI&SUBNET_MASK) ) {
		if (ifFrom!=IF_I2C ) return IF_I2C;
		else return IF_DROP;
	}

	if ((msg->header.destAddr&SUBNET_MASK) == (MYADDRX&SUBNET_MASK) ){
		if (ifFrom!=IF_XBEE ) return IF_XBEE;
		else return IF_DROP;
	}

	// else, sweep the table until you reach the matching subnetwork or the end
	while(rTable[i].destSubnet!=(0x42&(~SUBNET_MASK))){
		if ( rTable[i].destSubnet == (msg->header.destAddr&SUBNET_MASK) ) return rTable[i].ifTo;
		i++;
	}
	//if you reach the end, send to default destination
	return rTable[i].ifTo;
}




/*
 * Handles the forwarding of a message over the SuperBus network
 *
 *
 */
int sb_forward(sMsg *msg, E_IFACE ifFrom){
	switch (sb_route(msg, ifFrom)){
	case IF_XBEE :
		return Xbee_send(*msg);
		break;
	case IF_I2C :
		//return i2c_send(msg)
		break;
	case IF_DROP :
		return 0;
		break;
	case IF_LOCAL :
		iLast=(iLast+1)%SB_INC_MSG_BUF_SIZE;
		if (iFirst==iLast) iFirst=(iFirst+1)%SB_INC_MSG_BUF_SIZE; //"drop" oldest message if buffer is full

		memcpy(&(msgBuf[iLast]),msg, msg->header.size+sizeof(sGenericHeader));
		return (msgBuf[iLast].header.size + sizeof(sGenericHeader));
		break;
	default : return 0;
	}
	return 0;
}



int sb_printDbg(sb_Adress dest,char * str,int32_t i, uint32_t u){
	sMsg tmp;
	tmp.header.destAddr=dest;
	tmp.header.srcAddr=( (MYADDRX)==0?(MYADDRI):(MYADDRX) ) ;
	tmp.header.type=E_DEBUG;
	tmp.header.size=sizeof(sDebugPayload);
	tmp.payload.debug.i=i;
	tmp.payload.debug.u=u;
	strncpy((char *)&(tmp.payload.debug.msg),str,32);
	tmp.payload.debug.msg[31]=0; //strncpy does no ensure the null-termination, so we force it

	sb_send(&tmp);

	return 0;
}














