/*
 * lib_comm.h
 *
 *  Created on: 5 mai 2013
 *      Author: quentin
 */

#ifndef BN_CHECKSUM_H_
#define BN_CHECKSUM_H_

#include "messages.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint8_t checkSum(sMsg *msg);
void setSum(sMsg *msg);

#ifdef __cplusplus
}
#endif

#endif /* BN_CHECKSUM_H_ */
