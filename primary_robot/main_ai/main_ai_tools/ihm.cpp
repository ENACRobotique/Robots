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

eIhmLed ihmLedConvertRGBToEnum(sRGB rgb){
    if(rgb.red < 64 && rgb.green > 127 && rgb.blue < 64)
        return eIhmLed::LED_GREEN;
    if(rgb.red > 127 && rgb.green > 127 && rgb.blue < 64)
       return eIhmLed::LED_YELLOW;
    if(rgb.red < 64 && rgb.green < 64 && rgb.blue < 64)
       return eIhmLed::LED_OFF;

    logs << ERR << "Unkowned color";

    return eIhmLed::LED_OFF;
}

sRGB ihmLedConvertEnumToRgb(eIhmLed led){
    switch(led){
        case LED_GREEN:
            return {0, 255, 0};
        case LED_YELLOW:
            return {255, 255, 0};
        case LED_OFF:
            return {0, 0, 0};
    }

    logs << ERR << "Unkowned color";

    return {0, 0, 0};
}

void Ihm::receivedNewIhm(sIhmStatus &ihm){ //TODO check all value with arduino_io
    for(int i = 0 ; i < (int) ihm.nb_states ; i++){
        switch(ihm.states[i].id){
            case IHM_STARTING_CORD:
                list[eIhmElement::IHM_STARTING_CORD] = ihm.states[i].state.state_cord;
                logs << MES << "[IHM] ## scord: " << list[eIhmElement::IHM_STARTING_CORD];
                break;
            case IHM_MODE_SWITCH:
                list[eIhmElement::IHM_MODE_SWITCH] = ihm.states[i].state.state_switch;
                logs << MES << "[IHM] ## smode: " << list[eIhmElement::IHM_MODE_SWITCH];
                break;
            case IHM_LED:
                list[eIhmElement::IHM_LED] = ihmLedConvertRGBToEnum(ihm.states[i].state.color1);
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
    switch(id){
        case IHM_STARTING_CORD:
            msgOut.payload.ihmStatus.states[0].state.state_cord = (eIhmCord)state;
            break;
        case IHM_MODE_SWITCH:
            msgOut.payload.ihmStatus.states[0].state.state_switch = (eIhmSwitch) state;
            break;
        case IHM_LED:
            break;
        default:
            logs << ERR << "Unkown Ihm send";
            break;
    }

    bn_sendRetry(&msgOut, MAX_RETRIES);
    logs << MES << "[IHM] Sending new status id=" << id << ", state=" << state;
}
