/*
 * lib_sbDebug.c
 *
 *  Created on: 9 oct. 2013
 *      Author: quentin
 */

#ifndef LIB_SBDEBUG_C_
#define LIB_SBDEBUG_C_

#include "lib_superBus.h"
#include "lib_sbDebug.h"
#include "messages.h"
#include "network_cfg.h"
#include "tools.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>


//debug address
sb_Address debug_addr=ADDR_DEBUG_DFLT;


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

    char string[SB_MAX_PDU-sizeof(sGenericHeader)];

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
void sb_debugUpdateAddr(sMsg * msg){

    if (msg->header.type==E_DEBUG_SIGNALLING) {
         debug_addr=msg->header.srcAddr;
    }
    sb_printfDbg("dbgaddr updtated to %hx\n",msg->header.srcAddr);
}

/* sb_debugSignalling : sends the new debug address to dest. MUST be issued ONLY by the debugger.
 * Arguments :
 *  dest : address of the node whitch we want up update
 * Return value : like sb_send.
 */
int sb_debugSendAddr(sb_Address dest){
    sMsg msg;
    msg.header.destAddr=dest;
    msg.header.type=E_DEBUG_SIGNALLING;
    msg.header.size=0;
    return sb_send(&msg);

}

#endif /* LIB_SBDEBUG_C_ */
