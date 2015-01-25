/************************************** 

**************************************/


#include "lib_odo.h"
#include "../params.h"

//globals
int _pinIntOdo[NB_MOTORS],_pinSenOdo[NB_MOTORS];  //interruption pin, sense pin
volatile int _nbIncOdo[NB_MOTORS]={0};   //number of increments (signed)


void odoIsr1(){
    if (digitalRead(_pinIntOdo[0])) _nbIncOdo[0] += (digitalRead(_pinSenOdo[0])<<1) -1;
    else _nbIncOdo[0] -= (digitalRead(_pinSenOdo[0])<<1) -1;
}

#if NB_MOTORS > 2
void odoIsr2(){
    if (digitalRead(_pinIntOdo[1])) _nbIncOdo[1] += (digitalRead(_pinSenOdo[1])<<1) -1;
    else _nbIncOdo[1] -= (digitalRead(_pinSenOdo[1])<<1) -1;
}
#endif
//initialise the pins and sets the ISR
//requires : nothing
void odoInitHard(int pinInt[], int pinSen[]){
	for(int i=0;i<NB_MOTORS;i++)
	{
		_pinIntOdo[i]=pinInt[i];
		_pinSenOdo[i]=pinSen[i];
		pinMode(_pinIntOdo[i],INPUT);
		pinMode(_pinSenOdo[i],INPUT);
	}
    attachInterrupt(pinInt[0]-2, odoIsr1 , CHANGE); //particular case for arduino uno, cf reference
#if NB_MOTORS > 2
    attachInterrupt(pinInt[1]-2, odoIsr2 , CHANGE); //particular case for arduino uno, cf reference
#endif
}


//remove the ISR
void odoDeinit(){
    detachInterrupt(_pinIntOdo[0]-2); //particular case for arduino uno, cf reference
#if NB_MOTORS > 2
    detachInterrupt(_pinIntOdo[1]-2); //particular case for arduino uno, cf reference
#endif
}

int odoRead(int motor_index){
    int tmp=_nbIncOdo[motor_index];
    _nbIncOdo[motor_index]=0;
    return tmp;
}
