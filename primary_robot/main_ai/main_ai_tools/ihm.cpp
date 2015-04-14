/*
 * ihm.cpp
 *
 *  Created on: 23 févr. 2015
 *      Author: seb
 */

#include <ihm.h>

#include <iostream>
extern "C"{
#include <math.h>
}
Ihm::Ihm(): _starting_cord(CORD_OUT), _switch_color(SWITCH_OFF), _led(LED_OFF) {
    //TODO send message to get ihm status

}

Ihm::~Ihm() {
    // TODO Auto-generated destructor stub
}

void Ihm::receivedNewIhm(sIhmStatus &ihm){ //TODO check all value with arduino_io
    for(int i = 0 ; i < (int) ihm.nb_states ; i++){
        switch(ihm.states[i].id){
            case IHM_STARTING_CORD:
                if(ihm.states[i].state == 0) //TODO check
                    _starting_cord = CORD_OUT;
                else
                    _starting_cord = CORD_IN;
                cout << "[INFO] [IHM] ## scord: " << _starting_cord << endl;
                break;
            case IHM_MODE_SWICTH:
                if(ihm.states[i].state == 1)
                    _switch_color = SWITCH_ON;
                else
                    _switch_color = SWITCH_OFF;
                cout << "[INFO] [IHM] ## smode: " << _switch_color << endl;
                break;
            case IHM_LED:
                if(ihm.states[i].state == 0)
                    _led = LED_RED;
                else if(ihm.states[i].state == 1)
                    _led = LED_YELLOW;
                else
                    _led = LED_OFF;

                cout << "[INFO] [IHM] ## sled:" << endl;
                break;
            default:
                cout << "[WARNING] id not define or doesn't exist" << endl;
            break;
         }
     }
}

eIhmCord Ihm::getStartingCord(){
    return _starting_cord;
}

eIhmSwitch Ihm::getSwitchColr(){
    return _switch_color;
}

eIhmLed Ihm::getLed(){
    return _led;
}
