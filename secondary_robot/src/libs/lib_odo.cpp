/************************************** 

**************************************/


#include "lib_odo.h"
#include "params.h"

#if NB_MOTORS > 2
#error "Too many motors. You have 2 interruptions pin on arduino UNO"
#endif

//globals
int _pinIntOdo[NB_MOTORS],_pinSenOdo[NB_MOTORS];  //interruption pin, sense pin
volatile int _nbIncOdo[NB_MOTORS]={0};   //number of increments (signed)


void odoIsr1(){
    if (!digitalRead(_pinIntOdo[0])) _nbIncOdo[0] += (digitalRead(_pinSenOdo[0])<<1) -1;
    else _nbIncOdo[0] -= (digitalRead(_pinSenOdo[0])<<1) -1;
}

#if NB_MOTORS > 1
void odoIsr2(){
    if (digitalRead(_pinIntOdo[1])) _nbIncOdo[1] += (digitalRead(_pinSenOdo[1])<<1) -1;
    else _nbIncOdo[1] -= (digitalRead(_pinSenOdo[1])<<1) -1;
}
#endif
//initialise the pins and sets the ISR
//requires : nothing
void odoInitHard(int pinInt[], int pinSen[]){
	_pinIntOdo[0]=pinInt[0];
	_pinSenOdo[0]=pinSen[0];
	pinMode(_pinIntOdo[0],INPUT);
	pinMode(_pinSenOdo[0],INPUT);

    attachInterrupt(pinInt[0]-2, odoIsr1 , CHANGE); //particular case for arduino uno, cf reference
#if NB_MOTORS > 1
    _pinIntOdo[1]=pinInt[1];
	_pinSenOdo[1]=pinSen[1];
	pinMode(_pinIntOdo[1],INPUT);
	pinMode(_pinSenOdo[1],INPUT);
    attachInterrupt(pinInt[1]-2, odoIsr2 , CHANGE); //particular case for arduino uno, cf reference
#endif
}


//remove the ISR
void odoDeinit(){
    detachInterrupt(_pinIntOdo[0]-2); //particular case for arduino uno, cf reference
#if NB_MOTORS > 1
    detachInterrupt(_pinIntOdo[1]-2); //particular case for arduino uno, cf reference
#endif
}

int odoRead(int motor_index){
    int tmp=_nbIncOdo[motor_index];
    _nbIncOdo[motor_index]=0;
    return tmp;
}
