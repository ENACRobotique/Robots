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
	static int memValue[]={0,0,0};
	int Position = (myEnc.read()/2)%3;    //position du selecteur
	int Value = analogRead(Position+1);

	if(abs(Value-memValue[Position])>11)
	{
		char affich[16];
		double val;
		val=Value/1024.0;
		int pe=val*100;
		snprintf(affich,17,"CH%d : %d %%",Position,pe);
		afficher(affich);
		memValue[Position]=Value;
	}

	if(!digitalRead(RETOUR))
	{
		delay(DELAY_BOUNCE);	//anti rebond
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

//void analogRead(){
//
//}

sState sanalogRead={
    0,
    &initanalogRead,
    &deinitanalogRead,
    &testanalogRead
};


