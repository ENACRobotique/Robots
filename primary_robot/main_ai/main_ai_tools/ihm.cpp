/*
 * ihm.cpp
 *
 *  Created on: 23 févr. 2015
 *      Author: seb
 */

#include <ihm.h>

#include <iostream>

#include "tools.h"
#include "network_cfg.h"
#include "botNet_core.h"
extern "C"{
#include <math.h>
}

void Ihm::receivedNewIhm(sIhmStatus &ihm){ //TODO check all value with arduino_io
    for(int i = 0 ; i < (int) ihm.nb_states ; i++){
        switch(ihm.states[i].id){
            case IHM_STARTING_CORD:
                list[eIhmElement::IHM_STARTING_CORD] = ihm.states[i].state;
                logs << MES << "[IHM] ## scord: " << list[eIhmElement::IHM_STARTING_CORD];
                break;
            case IHM_MODE_SWICTH:
                list[eIhmElement::IHM_MODE_SWICTH] = ihm.states[i].state;
                logs << MES << "[IHM] ## smode: " << list[eIhmElement::IHM_MODE_SWICTH];
                break;
            case IHM_LED:
                list[eIhmElement::IHM_LED] = ihm.states[i].state;
                logs << MES << "[IHM] ## sled: " << list[eIhmElement::IHM_LED];
                break;
            default:
                logs << WAR << "id not define or doesn't exist";
            break;
         }
     }
}

void Ihm::sendIhm(const eIhmElement& id, const unsigned int& state){
    sMsg msgOut ;

    msgOut.header.destAddr = ADDRI_MAIN_IO;
    msgOut.header.type = E_IHM_STATUS;
    msgOut.header.size = 2 + 1*sizeof(*msgOut.payload.ihmStatus.states);

    msgOut.payload.ihmStatus.nb_states = 1;
    msgOut.payload.ihmStatus.states[0].id = id;
    msgOut.payload.ihmStatus.states[0].state = state;

    bn_sendRetry(&msgOut, MAX_RETRIES);
    logs << MES << "[IHM] Sending new status id=" << id << ", state=" << state;
}
