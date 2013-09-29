//Important notice !
//This file is an example
//Do not use "as is". This code must be pasted in your project and modified to match your needs.


#ifndef GENERIC_STATE_H_
#define GENERIC_STATE_H_


#include "state_type.h"


extern sState sGenericState;


sState* testGenericState();
void initGenericState(sState *prev);
void deinitGenericState(sState *next);


#endif /* GENERIC_STATE_H_ */
