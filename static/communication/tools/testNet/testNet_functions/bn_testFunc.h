/*
 * bn_testFunc.h
 *
 *  Created on: 16 janv. 2014
 *      Author: quentin
 */

#ifndef BN_TESTFUNC_H_
#define BN_TESTFUNC_H_

#include "../../../network_tools/bn_debug.h"
#include "../../../botNet/shared/botNet_core.h"

#ifdef __cplusplus
extern "C" {
#endif




int cbr_deamon();       // constant bit rate source, must run on the source of the flux.

// CBR management functions
int cbr_newFlux(sMsg *msg);
int cbr_controller(sMsg *msg);
int cbr_start(bn_Address server, bn_Address receiver,uint8_t fluxID, uint32_t period, uint8_t size,uint32_t number);
int cbrAcked_start(bn_Address server, bn_Address receiver,uint8_t fluxID, uint32_t period, uint8_t size,uint32_t number);
int cbr_printResults(bn_Address server, int8_t fluxID);
int cbr_reset(bn_Address server);
int cbr_stop(bn_Address server, int8_t fluxID);

int well_deamon(sMsg *msg);             // counts received test packets
int well_resquestStat(bn_Address server,bn_Address sender, int8_t fluxID);  // sends a result request
int well_reset(bn_Address server);  // sends a "delete everything" request

#ifdef __cplusplus
}
#endif

#endif /* BN_TESTFUNC_H_ */
