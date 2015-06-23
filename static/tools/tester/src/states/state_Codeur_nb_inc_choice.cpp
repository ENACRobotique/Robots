/*
 * state_Codeur_nb_inc_choice.cpp
 *
 *  Created on: 2015 juin 22
 *      Author: Fab
 */


#include "Arduino.h"
#include "state_Codeur_nb_inc_choice.h"
#include "state_Codeur.h"
#include "params.h"
#include "tools.h"
#include "state_types.h"
#include "lib_IHM.h"

#define MAX_INC_BY_TURN 32000
#define MIN_INC_BY_TURN 800

sState* testCodeur_nb_inc_choice(){
	static long temps_enc=0;
	static int pos_enc_old=myEnc.read();

	int pos_enc = myEnc.read();

	if(pos_enc!=pos_enc_old){
		if(millis()-temps_enc < DUREE_BIG_STEPS){
			pos_enc = pos_enc_old + 50 * (pos_enc - pos_enc_old);
		}
		else{
			pos_enc = pos_enc_old + 5 * (pos_enc - pos_enc_old);
		}
		pos_enc = CLAMP(MIN_INC_BY_TURN, pos_enc, MAX_INC_BY_TURN);
		temps_enc=millis();

		afficher("Nb inc: %d", pos_enc);
		myEnc.write(pos_enc);
		pos_enc_old=pos_enc;
	}

	if(!digitalRead(SELECT)){	//si on appui sur select:
		while(!digitalRead(SELECT));
		inc_t = pos_enc;
		return &sCodeur;
	}

	if(!digitalRead(RETOUR)){
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sCodeur);
	}

	return NULL;
}

void initCodeur_nb_inc_choice(sState *prev){
	myEnc.write(inc_t);
	afficher("Nb inc: %d", inc_t);
}

void deinitCodeur_nb_inc_choice(sState *next){
}

sState sCodeur_nb_inc_choice={
		0,
        &initCodeur_nb_inc_choice,
        &deinitCodeur_nb_inc_choice,
        &testCodeur_nb_inc_choice
};

