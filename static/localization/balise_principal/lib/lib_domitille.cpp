

#include "Arduino.h"
#include "lib_domitille.h"
#include "params.h"

//interruption pin, sense pin
int _pinInt;

//number of revolution
volatile unsigned int _nbTR=0;
volatile uint32_t prev_int=0,TR_mean_period=0,TR_lastDate=0;

// records of laste turn informations (to compute angles)
volatile int TR_iNext=0;    // where the next value will be written (by interruption)
volatile sTurnInfo TR_InfoBuf[TR_INFO_BUFFER_SIZE]={{0}};


volatile unsigned long filter_reg=0;

#if defined(BLINK_1TR)
volatile int led=0;
#endif

void domi_isr(){
    unsigned long time=micros();
    if ( (time-prev_int)<20) {      // because of the shape of the signal
        uint32_t period=time-TR_lastDate;
        TR_InfoBuf[TR_iNext].period=period;          // period of the previous turn
        TR_InfoBuf[TR_iNext].date=TR_lastDate;

        filter_reg= filter_reg- (filter_reg>>FILTER_SHIFT)+period;
        TR_mean_period=filter_reg>>FILTER_SHIFT;
        _nbTR++;

        TR_iNext=(TR_iNext+1)%TR_INFO_BUFFER_SIZE;
        TR_lastDate=time;// begin of the new turn

#ifdef BLINK_1TR
        led^=1;
        digitalWrite(PIN_DBG_LED,led);
#endif
    }
    prev_int=time;

}

//initialise the ISR
void domi_init(int pinInt){
    _pinInt=pinInt;
    pinMode(_pinInt,INPUT);
    attachInterrupt(pinInt-2, domi_isr, CHANGE); //particular case for arduino uno, cf reference
#ifdef BLINK_1TR
    pinMode(13,OUTPUT);

#endif
}

//remove the ISR
void domi_deinit(){
    detachInterrupt(_pinInt-2); //particular case for arduino uno, cf reference
}

