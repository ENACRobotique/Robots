/*
 * Xbee_API_arduino_drivers.c
 *
 *  Created on: 17 juin 2013
 *      Author: quentin
 */

#include "Arduino.h"
#include "node_cfg.h"

#include "Xbee_API_arduino_drivers.h"

void Xbee_rst(){

#ifdef XBEE_RST_PIN
    pinMode(XBEE_RST_PIN,OUTPUT);
    digitalWrite(XBEE_RST_PIN,HIGH);
    delay(10);
    digitalWrite(XBEE_RST_PIN,LOW);
#endif
    return ;
}

