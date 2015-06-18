/*
 * global_sync.h
 *
 *  Created on: 10 mai 2015
 *      Author: quentin
 */

#ifndef SRC_GLOBAL_SYNC_H_
#define SRC_GLOBAL_SYNC_H_

#include "messages.h"

#define SYNC_ARRAY_SIZE 5 // size of the global array that stores sync queries

void gs_receiveQuery(sMsg *msg);
int gs_testOne();
void gs_beaconStatus(eSyncStatus status);
/* gs_isBeaconRequested : must be tested. If returns 1, the turret MUST start synchronization with beacons
 * Rreturned value :
 *  1 if beacon synchronization is requested and not performed
 *  0 otherwise
 */
int gs_isBeaconRequested();
bn_Address gs_getBeaconQueryOrigin();
#endif /* SRC_GLOBAL_SYNC_H_ */
