/*
 * ihm.cpp
 *
 *  Created on: 23 févr. 2015
 *      Author: seb
 */

#include <ihm.h>

#include <iostream>
#include <cstring>
#include <cmath>

#include "tools.h"
#include "network_cfg.h"
#include "botNet_core.h"

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
            return {255, 170, 0};
        case LED_PURPLE:
        	return {153,0,153};
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
            case IHM_PRESENCE_1:
            	list[eIhmElement::IHM_PRESENCE_1] = ihm.states[i].state.state_presence;
            	logs << MES << "[IHM] ## spresence1 : "<< list[eIhmElement::IHM_PRESENCE_1];
            	break;
            case IHM_PRESENCE_2:
            	list[eIhmElement::IHM_PRESENCE_2] = ihm.states[i].state.state_presence;
            	logs << MES << "[IHM] ## spresence2 : "<< list[eIhmElement::IHM_PRESENCE_2];
            	break;
            case IHM_PRESENCE_3:
            	list[eIhmElement::IHM_PRESENCE_3] = ihm.states[i].state.state_presence;
            	logs << MES << "[IHM] ## spresence3 : "<< list[eIhmElement::IHM_PRESENCE_3];
            	break;
            case IHM_PRESENCE_4:
            	list[eIhmElement::IHM_PRESENCE_4] = ihm.states[i].state.state_presence;
            	logs << MES << "[IHM] ## spresence4 : "<< list[eIhmElement::IHM_PRESENCE_4];
            	break;
            case IHM_PRESENCE_5:
            	list[eIhmElement::IHM_PRESENCE_5] = ihm.states[i].state.state_presence;
            	logs << MES << "[IHM] ## spresence5 : "<< list[eIhmElement::IHM_PRESENCE_5];
            	break;
            default:
                logs << WAR << "[IHM] id not define or doesn't exist";
            break;
         }
     }
}

void Ihm::sendIhm(const eIhmElement& id, const unsigned int& state){
    sMsg msgOut ;
    memset(&msgOut, 0, sizeof(msgOut));

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
            msgOut.payload.ihmStatus.states[0].state.color1 = ihmLedConvertEnumToRgb((eIhmLed)state);
            msgOut.payload.ihmStatus.states[0].state.color2 = {0, 0, 0};
            msgOut.payload.ihmStatus.states[0].state.time1 = 10;
            msgOut.payload.ihmStatus.states[0].state.time2 = 0;
            msgOut.payload.ihmStatus.states[0].state.nb = 0;
            logs << WAR << "Send color";
            break;
        case IHM_PRESENCE_1:
        case IHM_PRESENCE_2:
        case IHM_PRESENCE_3:
        case IHM_PRESENCE_4:
        case IHM_PRESENCE_5:
        	msgOut.payload.ihmStatus.states[0].state.state_presence = (eIhmPresence) state;
        	break;
        default:
            logs << ERR << "Unkown Ihm send";
            break;
    }

    bn_sendRetry(&msgOut, MAX_RETRIES);
    logs << MES << "[IHM] Sending new status id=" << id << ", state=" << state;
}
