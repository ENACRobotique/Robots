/*
 * utils.h
 *
 *  Created on: 8 nov. 2013
 *      Author: quentin
 */

#ifndef BN_UTILS_H_
#define BN_UTILS_H_

#include "botNet_core.h"

typedef struct{
    bn_Address addr;
    uint32_t ping;
}sTraceInfo;

int sb_ping(bn_Address dest);
int sb_traceroute(bn_Address dest, sTraceInfo *retVals,int maxDpth, uint32_t timeout);


#endif /* LIB_SB_UTILS_H_ */
