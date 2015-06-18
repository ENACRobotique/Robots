

#include "Arduino.h"
#include "lib_domitille.h"
#include "params.h"
#include "shared/lib_synchro.h"

//interruption pin, sense pin
int _pinInt;
int _pinSpeed;

//number of revolution
volatile unsigned int _nbTR=0;
volatile uint32_t TR_mean_period=0,TR_lastDate=0;

// records of last turn informations (to compute angles)
volatile int TR_iNext=0;    // where the next value will be written (by interruption)
volatile sTurnInfo TR_InfoBuf[TR_INFO_BUFFER_SIZE]={{0}};


volatile unsigned long filter_reg=0;

#if defined(BLINK_1TR)
volatile int led=0;
#endif

void domi_isr(){
    static unsigned long prev_int=0;
    static unsigned long prev_duration=0;
    volatile unsigned long time=micros();
    if ( (time-prev_int)> (prev_duration + (prev_duration>>1))) {      // because of the shape of the signal
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
    prev_duration=time-prev_int;
    prev_int=time;

}

//initialise the ISR
void domi_init(int pinInt, int pinSpeed){
    _pinSpeed=pinSpeed;
    _pinInt=pinInt;
    pinMode(_pinInt,INPUT);
    attachInterrupt(pinInt-2, domi_isr, CHANGE); //particular case for arduino uno, cf reference
#ifdef BLINK_1TR
    pinMode(13,OUTPUT);
#endif

    pinMode(_pinSpeed,OUTPUT);
#ifdef SYNC_WIRED
    analogWrite(_pinSpeed,SPEED_20HZ);
#elif defined(SYNC_WIRELESS)
    analogWrite(_pinSpeed,SPEED_HIGH);
#endif
}

//remove the ISR
void domi_deinit(){
    detachInterrupt(_pinInt-2); //particular case for arduino uno, cf reference
}

void domi_setspeed(int speed){
    analogWrite(_pinSpeed,speed);
}
