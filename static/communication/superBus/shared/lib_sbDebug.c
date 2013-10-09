/*
 * lib_sbDebug.c
 *
 *  Created on: 9 oct. 2013
 *      Author: quentin
 */

#ifndef LIB_SBDEBUG_C_
#define LIB_SBDEBUG_C_

#include "lib_superBus.h"
#include "messages.h"
#include "tools.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>


//debug address
sb_Address debug_addr=ADDRX_DEBUG;


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
void sb_debugAddrSignal(sMsg * msg){

    if (msg->header.type==E_DEBUG_SIGNALLING) {
         debug_addr=msg->header.srcAddr;
    }
}

#endif /* LIB_SBDEBUG_C_ */
