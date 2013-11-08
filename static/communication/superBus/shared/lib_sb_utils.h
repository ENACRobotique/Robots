/*
 * lib_sb_utils.h
 *
 *  Created on: 8 nov. 2013
 *      Author: quentin
 */

#ifndef LIB_SB_UTILS_H_
#define LIB_SB_UTILS_H_

#include "lib_superBus.h"

typedef struct{
    sb_Address addr;
    uint32_t ping;
}sTraceInfo;

int ping(sb_Address dest);
int traceroute(sb_Address dest, sTraceInfo *retVals,int maxDpth, uint32_t timeout);


#endif /* LIB_SB_UTILS_H_ */
