/*
 * state_%STATENAME%.cpp
 *
 *  Created on: %DATE%
 *      Author: %AUTHOR%
 */


#include "Arduino.h"
#include "state_%STATENAME%.h"
%INCLUDES%


sState* test%STATENAME%(){
        // Your code here !
        return NULL;
    }

void init%STATENAME%(sState *prev){
        // Your code here !
    }

void deinit%STATENAME%(sState *next){
        // Your code here !
    }

sState s%STATENAME%={
        %FLAGS%
        &init%STATENAME%,
        &deinit%STATENAME%,
        &test%STATENAME%
};

