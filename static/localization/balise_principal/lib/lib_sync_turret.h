/*
 * lib_sync_turret.h
 *
 *  Created on: 18 mars 2014
 *      Author: quentin
 */

#ifndef LIB_SYNC_TURRET_H_
#define LIB_SYNC_TURRET_H_

#include "../../../communication/botNet/shared/botNet_core.h"

int sync_beginElection(bn_Address addr);
int sync_sendData(bn_Address addr);
int sync_sendEnd(bn_Address addr);
#endif /* LIB_SYNC_TURRET_H_ */
