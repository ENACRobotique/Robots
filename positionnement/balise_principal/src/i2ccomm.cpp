/*
 * i2ccomm.cpp
 *
 *  Created on: 9 mai 2013
 *      Author: quentin
 */

#include "i2ccomm.h"
#include "Wire.h"
#include "lib_domitille.h"
#include "messages.h"
#include "params.h"
#include "Arduino.h"

void requestHandler(){

 uint32_t tab[3];
 tab[1]=mesTab[1];
 tab[0]=mesTab[0];
 tab[2]=domi_meanPeriod();

 Wire.write((uint8_t *)tab,sizeof(tab));


}


