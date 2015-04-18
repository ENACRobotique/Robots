/*
 * main.c
 *
 *  Created on: 18 avr. 2015
 *      Author: quentin
 */

#include "data.h"
#include "shared/lib_synchro_wire.h"

int main(){

    int i=0;
    for (i=0; i<sizeof(measures)/sizeof(wsMeasure_t); i++){
        wiredSync_intermediateCompute((i+1)*1000000,measures[i].date);
    }
    wiredSync_finalCompute(0);

    return 0;
}
