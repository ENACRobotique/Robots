/*
 * lib_laser_arduino.cpp
 *
 *  Created on: 18 mai 2014
 *      Author: quentin
 */

#ifdef ARCH_328P_ARDUINO

#include "Arduino.h"
#include "../shared/lib_int_laser.h"
#include "../src/params.h"

void laser_arduino_IntHand0(){ //interrupt handler, puts the time in the rolling buffer
  //new! debouce, will hide any interruption happening less than DEBOUNCETIME µsec after the last registered interruption
    unsigned long time=micros();//mymicros();
    if ( time > (buf0.buf[(buf0.index-1)&7]+LASER_DEBOUNCETIME) ){
        buf0.buf[buf0.index]=time;
        buf0.index++;
        buf0.index&=7;
    }
    EIFR = BIT(0);
}

void laser_arduino_IntHand1(){
  //new! debouce, will hide any interruption happening less than DEBOUNCETIME µsec after the last registered interruption
    unsigned long time=micros();//mymicros();
    if ( time > (buf1.buf[(buf1.index-1)&7]+LASER_DEBOUNCETIME) ){
        buf1.buf[buf1.index]=time;
        buf1.index++;
        buf1.index&=7;
    }
    EIFR = BIT(1);
}


void laser_arduino_Intinit(){

    pinMode(0+2,INPUT);  // trick for the arduino UNO only
    attachInterrupt(0, laser_arduino_IntHand0 ,CHANGE);
    pinMode(1+2,INPUT);  // trick for the arduino UNO only
    attachInterrupt(1, laser_arduino_IntHand1 ,CHANGE);
}


void laser_arduino_Intdeinit(){
    detachInterrupt(0);
    detachInterrupt(1);
}

#endif
