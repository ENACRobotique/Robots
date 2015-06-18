/*
 * state_Analog_read.cpp
 *
 *  Created on: 2015 juin 18
 *      Author: Fab
 */


#include "Arduino.h"
#include "state_Analog_read.h"
#include "params.h"
#include "tools.h"
#include "state_types.h"
#include "state_Menu_principal.h"
#include "lib_IHM.h"


sState* testAnalog_read(){
        // Your code here !
        return NULL;
    }

void initAnalog_read(sState *prev){
        // Your code here !
    }

void deinitAnalog_read(sState *next){
        // Your code here !
    }

sState sAnalog_read={
		0,
        &initAnalog_read,
        &deinitAnalog_read,
        &testAnalog_read
};

