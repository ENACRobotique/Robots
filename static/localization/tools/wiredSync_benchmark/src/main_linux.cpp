/*
 * main.c
 *
 *  Created on: 18 avr. 2015
 *      Author: quentin
 */

#include "data.h"
#include "shared/lib_synchro_wire.h"

int main(){

    unsigned int i=0;
    for (i=0; i<sizeof(measures)/sizeof(wsMeasure_t); i++){
        wiredSync_intermediateCompute(i*1000000+1200000,measures[i].date);
    }
    wiredSync_finalCompute(0);
    printf("number of iterations  : %d\n",i);

    return 0;
}
