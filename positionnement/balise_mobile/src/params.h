/*
 * params.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "messages.h"

typedef enum{
    CHANNEL,
    SYNC,
    GAME
} mainState;

#define MYADDR ADDR_MOBILE_2

#define SYNC_TOL 8 //in µs. max desync tolerated

#define DEBUG

#endif /* PARAMS_H_ */
