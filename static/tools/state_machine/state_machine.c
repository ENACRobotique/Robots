//Important notice !
//This file is an example
//Do not use "as is". This code must be pasted in your project and modified to match your needs.

#include "state_type.h"
#include "generic_state.h"

//current state. Must be initialized with a pointer to the first state
sState *current=&sGenericState;


/*function that runs the state machine. must be always called.
 * example for arduino : the code in loop() must be the same as the following one :
 *
 */
void smLoop(){

	// simple functions that will appear in more than one state. Add you own blocks if needed
    if (current->flag & BIT(E_GENERICBLOCK) ) genericBlock();


    //do not modify the following code ! core of the state_machine
    sState *next;
    if (current->testFunction){
        if ( (next=(current->testFunction()) ) ) {      //if the test fuction returns a non-zero value
            if (current->deinit) current->deinit(next); //we call deinit of the current state with the pointer to next state
            if (next->init) next->init(current);        //we call init of the next state with the pointer of current state
            current=next;                               //we set the new state
        }
    }
}
