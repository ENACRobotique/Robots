/*
 * state_Codeur.cpp
 *
 *  Created on: 2015 juin 22
 *      Author: Fab
 */


#include "Arduino.h"
#include "state_Codeur.h"
#include "params.h"
#include "tools.h"
#include "state_types.h"
#include "state_Menu_principal.h"
#include "lib_IHM.h"
#include "lib_odo.h"
#include "state_Codeur_nb_inc_choice.h"

#define FILTER_SHIFT 2

#define NB_VISU 4
int inc_t = 3200;

sState* testCodeur(){
	static int nbInc_filtered=0;
	static unsigned long prev_time = millis();
	if(millis()-prev_time > 249){
		int Position = myEnc.read(); //lecture de l'encodeur pour choisir le mode de visualisation

		//filtre passe bas : nb_filtered = a*nb_now + (1-a)*last_nb		avec a entre 0 et 1.
		int nbInc_now = odoRead();
		nbInc_filtered = nbInc_filtered - (nbInc_filtered >> FILTER_SHIFT) + nbInc_now;
		nbInc_now = nbInc_filtered >> FILTER_SHIFT;

		switch (Position)		//et on va dans le bon état
		{
			case 0:{		//en tours par secondes
				int N_ts = (800.0*nbInc_now)/inc_t;			//800 = 100 * (1000/250) * 2(int sur voie A seulement)
				int ent = abs(N_ts / 100);
				int dec = abs(N_ts - 100*ent);
				char sign = ' ';
				if(N_ts >> 15) sign = '-';
				afficher("N(t/s)= %c%d,%d",sign,ent,dec);
				break; }
			case 1:{		//en rd/s
				int W_rs = (800.0*nbInc_now*TWO_PI)/inc_t;	//800 = 100 * (1000/250) * 2(int sur voie A seulement)
				int ent = W_rs / 100;
				int dec = abs(W_rs - 100*ent);
				afficher("W(r/s)= %d,%d",ent,dec);
				break; }
			case 2:{
				int N_ds = (2880.0*nbInc_now)/inc_t;		//2880 = 360° * (1000/250) * 2(int sur voie A seulement)
				afficher("N(d/s)= %d",N_ds);
				break; }
			case 3:{
				int N_is = nbInc_now*8;						//8 = (1000/250) * 2(int sur voie A seulement)
				afficher("nb_inc/s= %d",N_is);
				break; }
		}
		prev_time = millis();
	}

	if(!digitalRead(SELECT)){	//si on appui sur select:
		while(!digitalRead(SELECT));
		return &sCodeur_nb_inc_choice;
	}

	if(!digitalRead(RETOUR)){
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_principal);
	}

	return NULL;
}

void initCodeur(sState *prev){
	odoInitHard(CODERINT,CODERSTATE);
	myEnc.setLimits(0,NB_VISU-1);
	myEnc.write(0);
	myEnc.setMultiplicators(1,1);
//	mf_init(&mf, 15, 0);
}

void deinitCodeur(sState *next){
	odoDeinit();
}

sState sCodeur={
		0,
        &initCodeur,
        &deinitCodeur,
        &testCodeur
};

