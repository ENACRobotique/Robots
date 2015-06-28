/*
 * state_Analog_read.cpp
 *
 *  Created on: 2015 juin 18
 *      Author: Fab
 */


#include "Arduino.h"
#include "state_Analog_read.h"
#include "params.h"
#include "tools.h"
#include "state_types.h"
#include "state_Menu_principal.h"
#include "lib_IHM.h"

sState* testAnalog_read(){
        static unsigned long last_time = millis();

        if(millis() - last_time > 200){		//Il ne sert a rien de rafraichir trop souvent.
        	int value = analogRead(AREAD);
        	int result = map(value,0,1023,0,1540);		//le pont diviseur divise ~ par 3, et la ref est à 5V.
        	int ent = result/100;
        	int dec = result - 100*ent;
        	afficher(0,1,"U = %d,%d V",ent,dec);
        	last_time = millis();
        }

        if(!digitalRead(RETOUR)){
			delay(DELAY_BOUNCE);	//anti rebond
			while(!digitalRead(RETOUR));	//attente du relachement du bouton
			return(&sMenu_principal);
		}
        return NULL;
    }

void initAnalog_read(sState *prev){
        // Your code here !
    }

void deinitAnalog_read(sState *next){
        // Your code here !
    }

sState sAnalog_read={
		0,
        &initAnalog_read,
        &deinitAnalog_read,
        &testAnalog_read
};
