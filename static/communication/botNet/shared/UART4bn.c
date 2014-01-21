/*
 * UART4bn.c
 *
 *  Created on: 21 janv. 2014
 *      Author: quentin
 */

#include "UART4bn.h"

#if MYADDRU


// Just because, CF http://gustedt.wordpress.com/2010/11/29/myth-and-reality-about-inline-in-c99/
extern inline int UART_receive(sMsg *pRet);
extern inline int UART_send(sMsg *msg);

#endif
