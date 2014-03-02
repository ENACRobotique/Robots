/*
 * lib_bnDebug.h
 *
 *  Created on: 9 oct. 2013
 *      Author: quentin
 */

#ifndef BN_DEBUG_H_
#define BN_DEBUG_H_

#include "messages.h"

#ifdef __cplusplus
extern "C" {
#endif

//useful macro
#ifndef MIN
#define MIN(m, n) (m)>(n)?(n):(m)
#endif
#ifndef MAX
#define MAX(m, n) (m)<(n)?(n):(m)
#endif

extern volatile bn_Address debug_addr;

int bn_printDbg(const char * str);
int bn_printfDbg(const char *format, ...) __attribute__ ((format (printf, 1, 2)));
void bn_debugUpdateAddr(sMsg * msg);
int bn_debugSendAddr(bn_Address dest);


#ifdef __cplusplus
}
#endif

#endif /* LIB_SBDEBUG_H_ */
