/************************************** 

**************************************/


#include "lib_odo.h"

//globals
int _pinIntOdo,_pinSenOdo;  //interruption pin, sense pin
volatile int _nbIncOdo=0;   //number of increments (signed)


void odoIsr(){
    _nbIncOdo += (digitalRead(_pinSenOdo)<<1) -1;
}

//initialise the pins and sets the ISR
//requires : nothing
void odoInitHard(int pinInt, int pinSen){
    _pinIntOdo=pinInt;
    _pinSenOdo=pinSen;
    pinMode(_pinIntOdo,INPUT);
    pinMode(_pinSenOdo,INPUT);
    attachInterrupt(pinInt-2, odoIsr , RISING); //particular case for arduino uno, cf reference
}


//remove the ISR
void odoDeinit(){
    detachInterrupt(_pinIntOdo-2); //particular case for arduino uno, cf reference
}

int odoRead(){
    int tmp;
    tmp=_nbIncOdo;
    _nbIncOdo=0;
    return tmp;
}
