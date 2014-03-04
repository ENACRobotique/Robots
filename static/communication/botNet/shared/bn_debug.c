/*
 * debug.c
 *
 *  Created on: 9 oct. 2013
 *      Author: quentin
 */


// config files
#include "messages.h"
#include "network_cfg.h"
#include "node_cfg.h"

// other required libraries
#include "../../../global_errors.h"

// botNet specific libraries
#include "botNet_core.h"
#include "bn_debug.h"

// standard libraries
#include <stdarg.h>
#include <stdio.h>
#include <string.h>


//debug address
volatile bn_Address debug_addr=ADDR_DEBUG_DFLT;


/* bn_printDbg : sends a fixed string in a message to the address debug_addr.
 * Arguments :
 *  str : pointer to the string to send
 * Return value :
 *  number of bytes written/send
 *  <0 if error (check error code)
 *
 * Remark : this will blindly shorten the string if the latter was too big.
 */
int bn_printDbg(const char *str){
    int ret;
    sMsg tmp;

    //do not send anything if the debug address is not defined
    if (debug_addr==0) return -ERR_BN_UNKNOWN_ADDR;

    tmp.header.destAddr=debug_addr;
    tmp.header.type=E_DEBUG;
    tmp.header.size=MIN(strlen(str)+1 , BN_MAX_PDU-sizeof(sGenericHeader));
    strncpy((char *)tmp.payload.data , str , BN_MAX_PDU-sizeof(sGenericHeader)-1);
    tmp.payload.debug[tmp.header.size-1]=0; //strncpy does no ensure the null-termination, so we force it

    ret = bn_send(&tmp);
    if(ret > sizeof(tmp.header)){
        ret -= sizeof(tmp.header);
    }
    else{
        ret = -1;
    }
    return ret;
}

/* bn_printfDbg : sends a string in a message to the address debug_addr unsing a printf-like formatting.
 * Arguments :
 *  format : see printf documentation
 * Return value :
 *  number of bytes written/send
 *  <0 if error (check error code)
 *
 * Remark : this will blindly shorten the string if the latter was too big.
 */
int bn_printfDbg(const char *format, ...){
    char string[BN_MAX_PDU-sizeof(sGenericHeader)];

    va_list ap;

    va_start(ap, format);
    vsnprintf((char *)string, BN_MAX_PDU-sizeof(sGenericHeader), format, ap);
    va_end(ap);

    return bn_printDbg(string);
}


/* bn_debugAddrSignal : modifies the local debug address. This should be used on receiving a signalling message of type E_DEBUG_SIGNALLING
 * Arguments :
 *  msg : pointer to the received message
 * Return value : none
 */
void bn_debugUpdateAddr(sMsg * msg){
    if (msg->header.type==E_DEBUG_SIGNALLING) {
         debug_addr=msg->header.srcAddr;
         bn_printfDbg("dbgaddr updated to %hx\n",msg->header.srcAddr);
    }
}

/* bn_debugSignalling : sends the new debug address to dest. MUST be issued ONLY by the debugger.
 * Arguments :
 *  dest : address of the node whitch we want up update
 * Return value : like bn_send.
 */
int bn_debugSendAddr(bn_Address dest){
    sMsg msg;
    msg.header.destAddr=dest;
    msg.header.type=E_DEBUG_SIGNALLING;
    msg.header.size=0;
    return bn_send(&msg);
}
