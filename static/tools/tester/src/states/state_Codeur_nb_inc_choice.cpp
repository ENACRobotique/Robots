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
	static int pos_enc_old=myEnc.read();

	int pos_enc = myEnc.read();

	if(pos_enc!=pos_enc_old){
		afficher("Nb inc: %d", pos_enc);
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
	myEnc.setLimits(MIN_INC_BY_TURN,MAX_INC_BY_TURN);
	myEnc.write(inc_t);
	myEnc.setMultiplicators(5,50);
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

