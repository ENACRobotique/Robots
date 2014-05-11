/*
 * Xbee_API_arduino_drivers.c
 *
 *  Created on: 17 juin 2013
 *      Author: quentin
 */

#ifdef ARCH_LM4FXX

#include "node_cfg.h"

#include "Xbee_API_lm4fxx_drivers.h"
#include "time.h"
#include "inc/lm4f120h5qr.h"

void Xbee_rst(){
#if defined(XBEE_RST_PIN) && defined(XBEE_RST_PORT)
    GPIOPinTypeGPIOOutput(XBEE_RST_PORT,XBEE_RST_PIN);
    GPIOPinWrite(XBEE_RST_PORT,XBEE_RST_PIN,XBEE_RST_PIN);
    delay(10);
    GPIOPinWrite(XBEE_RST_PORT,XBEE_RST_PIN,0);
#endif
    return ;
}

#endif
