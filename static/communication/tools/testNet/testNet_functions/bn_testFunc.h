/*
 * bn_testFunc.h
 *
 *  Created on: 16 janv. 2014
 *      Author: quentin
 */

#ifndef BN_TESTFUNC_H_
#define BN_TESTFUNC_H_

#include "../../../botNet/shared/bn_debug.h"
#include "../../../botNet/shared/botNet_core.h"

enum {
    E_CBR_START,
    E_CRB_START_ACK,
    E_CBR_STOP,
    E_CBR_STATS,
    E_CBR_RESET
};


enum {
    E_WELL_STATS,
    E_WELL_RESET
};


typedef struct {
    bn_Address receiver;
    uint8_t fluxID;
    uint32_t period;        // period of sending
    uint32_t sw;            // memory to store a stopwatch
    uint8_t size;           // size of the paylod of the packets
    uint32_t number;        // amount of packet left
} sFluxDescriptor;


int cbr_deamon(sMsg *msg);       // constant bit rate source

int cbr_start(bn_Address server, bn_Address receiver,uint8_t fluxID, uint32_t period, uint8_t size,uint32_t number);
int cbrAcked_start(bn_Address server, bn_Address receiver,uint8_t fluxID, uint32_t period, uint8_t size,uint32_t number);
int cbr_printResults(bn_Address server);
int cbr_reset(bn_Address);

int well_deamon(sMsg *msg);             // counts received test packets
int well_printStat(bn_Address server);  // sends back the results as a debug message


#endif /* BN_TESTFUNC_H_ */
