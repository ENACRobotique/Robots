/****************************************************************
gestion du temps (synchro)

pour ATMEGA 328P
****************************************************************/


#include "Arduino.h"


volatile unsigned long _microsOffset=0;
volatile unsigned long _millisOffset=0;



void setMicrosOffset(unsigned long offset){
  _microsOffset=offset;
  _millisOffset=offset/1000;
}

void setMillisOffset(long offset){
  _millisOffset=offset;
}

