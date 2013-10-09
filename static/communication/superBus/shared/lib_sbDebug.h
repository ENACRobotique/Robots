/*
 * lib_sbDebug.h
 *
 *  Created on: 9 oct. 2013
 *      Author: quentin
 */

#ifndef LIB_SBDEBUG_H_
#define LIB_SBDEBUG_H_

#include "messages.h"


int sb_printDbg(const char * str);
int sb_printfDbg(char *format, ...);
void sb_debugAddrSignal(sMsg * msg);

#endif /* LIB_SBDEBUG_H_ */
