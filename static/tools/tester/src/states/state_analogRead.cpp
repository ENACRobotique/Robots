/*
 * state-blink.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include "Arduino.h"
#include "../params.h"
#include "../tools.h"
#include "state_types.h"

#include "state_Menu_principal.h"
#include "state_analogRead.h"
#include "state_blink.h"

// test;

sState* testanalogRead(){
	static int memValue[]={0};
	int Value[] = {analogRead(0)};
	int lentab=(sizeof(Value)/sizeof(int));
	int Position = (myEnc.read()/2)%lentab;
	if(Value!=memValue)
	{
		char affich[16];
		for(int i=Position;i<min(Position+1,lentab);i++)
		{
			snprintf(affich,17,"CH%d:%d%%  ",i,Value[i]/1024);
		}
		afficher(affich);
		for(int i=0;i<1;i++)
		{
			memValue[i]=Value[i];
		}
	}

	if(!digitalRead(RETOUR))
	{
		delay(3);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_principal);
	}
    return NULL;
}
void initanalogRead(sState *prev){
	myEnc.write(0);
}
void deinitanalogRead(sState *next){

}

void analogRead(){

}

sState sanalogRead={
    0,
    &initanalogRead,
    &deinitanalogRead,
    &testanalogRead
};


