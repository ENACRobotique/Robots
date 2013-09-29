//Important notice !
//This file is an example
//Do not use "as is". This code must be pasted in your project and modified to match your needs.


#include "generic_state.h"

/* testGenericState :
 * Argument : none
 * Return Value : pointer to next state, 0 if the state machine must remain in the current state.
 *
 * This function will be called at every loop in this state.
 * It must perform the tests and return, if needed, the next state
 * It must return 0 to stay in the same state
 */
sState* testGenericState(){
    return 0;
}

/* initGenericState :
 * Argument : pointer to the previous state
 * Return Value : none.
 *
 * This function will be called EVERY TIME we ENTER the state (no if we stay in the state).
 * It must perform the initializations required for this particular state.
 * It can take into account for this the previous state
 */
void initGenericState(sState *prev){

}

/* deinitGenericState :
 * Argument : pointer to the next state
 * Return Value : none.
 *
 * This function will be called EVERY TIME we LEAVE the state (no if we stay in the state).
 * It must perform the de-initializations required for this particular state.
 * It can take into account for this the next state
 */
void deinitGenericState(sState *next){

}

/*
 * sGenericState :
 *  This structure defines a state.
 *  The E_BLOCK flags are used to perform action regularly
 */
sState sGenericState={
    0 , // or E_BLOCK1 | E_BLOCK 3 ...
    &initGenericState,
    &deinitGenericState,
    &testGenericState
};
