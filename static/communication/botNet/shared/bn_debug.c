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
#include "roles.h"

// other required libraries
#include "../../../global_errors.h"

// botNet specific libraries
#include "botNet_core.h"
#include "bn_debug.h"

// standard libraries
#include <stdarg.h>
#include <stdio.h>
#include <string.h>


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

//    tmp.header.destAddr=addr; role_send() => useless
    tmp.header.type=E_DEBUG;
    tmp.header.size=MIN(strlen(str)+1 , BN_MAX_PDU-sizeof(sGenericHeader));
    strncpy((char *)tmp.payload.data , str , BN_MAX_PDU-sizeof(sGenericHeader)-1);
    tmp.payload.debug[tmp.header.size-1]=0; //strncpy does no ensure the null-termination, so we force it

    ret = role_send(&tmp);
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

/* bn_debugSignalling : sends the new debug address to dest. MUST be issued ONLY by the debugger.
 * Arguments :
 *  dest : address of the node whitch we want up update
 * Return value : like bn_send.
 */
int bn_debugSendAddr(bn_Address dest){
    sMsg msg;

    msg.header.type = E_ROLE_SETUP;
    msg.header.destAddr = dest;
    msg.header.size = 2 + 4*2;
    msg.payload.roleSetup.nb_steps = 2;
    // step #0 (overrides any previous debug setup on the remote node)
    msg.payload.roleSetup.steps[0].step = UPDATE_ACTIONS;
    msg.payload.roleSetup.steps[0].type = E_DEBUG;
    msg.payload.roleSetup.steps[0].actions.sendTo.first = ROLE_DEBUG;
    msg.payload.roleSetup.steps[0].actions.sendTo.second = ROLE_UNKNOWN;
    msg.payload.roleSetup.steps[0].actions.relayTo.n1 = ROLE_UNKNOWN;
    msg.payload.roleSetup.steps[0].actions.relayTo.n2 = ROLE_UNKNOWN;
    // step #1 (I will be the default debug node for this remote one)
    msg.payload.roleSetup.steps[1].step = UPDATE_ADDRESS;
    msg.payload.roleSetup.steps[1].role = ROLE_DEBUG;
    msg.payload.roleSetup.steps[1].address = MYADDR;

    return bn_send(&msg);
}
