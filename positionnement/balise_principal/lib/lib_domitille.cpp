

#include "Arduino.h"
#include "lib_domitille.h"

//interruption pin, sense pin
int _pinInt;

//number of revolution
volatile unsigned int _nbTR=0;
volatile unsigned long prev_int=0,last_TR=0,TR_period=0,TR_mean_period=0;
#define FILTER_SHIFT 4
volatile unsigned long filter_reg=0;

void domi_isr(){
    unsigned long time=micros();
    if ( (time-prev_int)<32) {
        TR_period=time-last_TR;
        filter_reg= filter_reg- (filter_reg>>FILTER_SHIFT) +TR_period;
        TR_mean_period=filter_reg>>FILTER_SHIFT;
        last_TR=time;
        _nbTR++;
    }
    prev_int=time;

}

//initialise the ISR
void domi_init(int pinInt){
    _pinInt=pinInt;
    pinMode(_pinInt,INPUT);
    attachInterrupt(pinInt-2, domi_isr, CHANGE); //particular case for arduino uno, cf reference
}

//remove the ISR
void domi_deinit(){
    detachInterrupt(_pinInt-2); //particular case for arduino uno, cf reference
}


