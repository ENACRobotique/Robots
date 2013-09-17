/*
 * lib_comm.h
 *
 *  Created on: 5 mai 2013
 *      Author: quentin
 */

#ifndef LIB_COMM_H_
#define LIB_COMM_H_

#include "messages.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint8_t checksumHead(sGenericHeader *pt);
uint8_t cbChecksumHead(uint8_t *pt,uint8_t size, uint8_t lastB);
uint8_t calSsumHead(uint8_t *pt);
uint8_t checksumPload(sMsg *msg);
uint8_t calcSumPload(uPayload *pt,int size);
void setSum(sMsg *msg);

#ifdef __cplusplus
}
#endif

#endif /* LIB_COMM_H_ */
