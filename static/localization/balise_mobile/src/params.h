/*
 * params.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef PARAMS_H_
#define PARAMS_H_


typedef enum{
    S_BEGIN,
    S_CHANNEL,        // Initial channel selection
    S_SYNC_ELECTION,  // Sync laser interruption election
    S_SYNC_MEASURES,  // Clock drift measurement
    S_SYNCED,         // Waiting until everybody is synced (message from turret tells us)
    S_GAME            // Game mode
} mainState;


#define DEBUG

#endif /* PARAMS_H_ */
