/*
 * UART4sb.h
 *
 *  Created on: 1 nov. 2013
 *      Author: quentin
 */

#ifndef UART4SB_H_
#define UART4SB_H_

#include "messages.h"
#include "lib_UART_framing.h"

#ifdef __cplusplus
extern "C" { //to enable use in both C projects an C++ projects
#endif

// UART_init() is located in lib_UART_framing.h

inline int UART_receive(sMsg *pRet){
    return UART_readFrame(pRet,sizeof(sMsg));
}

inline int UART_send(sMsg *msg){
    return UART_writeFrame(msg,sizeof(sGenericHeader)+msg->header.size);
}

#ifdef __cplusplus
}
#endif

#endif /* UART4SB_H_ */
