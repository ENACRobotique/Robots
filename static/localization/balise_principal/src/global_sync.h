/*
 * global_sync.h
 *
 *  Created on: 10 mai 2015
 *      Author: quentin
 */

#ifndef SRC_GLOBAL_SYNC_H_
#define SRC_GLOBAL_SYNC_H_

#include "messages.h"

#define SYNC_ARRAY_SIZE 10 // size of the global array that stores sync queries

void gs_receiveQuery(sMsg *msg);
int gs_loop();

#endif /* SRC_GLOBAL_SYNC_H_ */
