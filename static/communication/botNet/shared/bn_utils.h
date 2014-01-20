/*
 * utils.h
 *
 *  Created on: 8 nov. 2013
 *      Author: quentin
 */

#ifndef BN_UTILS_H_
#define BN_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "botNet_core.h"

typedef struct{
    bn_Address addr;
    uint32_t ping;
}sTraceInfo;

int bn_ping(bn_Address dest);
int bn_traceroute(bn_Address dest, sTraceInfo *retVals,int maxDpth, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* LIB_SB_UTILS_H_ */
