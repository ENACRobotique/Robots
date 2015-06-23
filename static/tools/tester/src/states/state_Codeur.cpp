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

#define NB_VISU 3
int inc_t = 3200;

sState* testCodeur(){
	static unsigned long prev_time = millis();
	static int memPosition;
	int Position = myEnc.read();
	if(millis()-prev_time > 499){
		//////////////////////////////// lecture de l'encodeur pour choisir le mode de visualisation
		if(Position != memPosition){
			if (Position != CLAMP(0,Position,NB_VISU)){
				Position = CLAMP(0,Position,NB_VISU);
				myEnc.write(Position);
			}
		    memPosition=Position;
		}
		////////////////////////////////

		int nbInc = odoRead();
		//rajouter un median filter pour lisser les valeurs!
		switch (Position)		//et on va dans le bon état
		{
			case 0:{		//en tours par secondes
				int N_ts = (400.0*nbInc)/inc_t;			//400 = 100 * (1000/500) * 2(int sur voie A seulement)
				int ent = abs(N_ts / 100);
				int dec = abs(N_ts - 100*ent);
				char sign = ' ';
				if(N_ts >> 15) sign = '-';
				afficher("N(t/s)= %c%d,%d",sign,ent,dec);
				break; }
			case 1:{		//en rd/s
				int W_rs = (400.0*nbInc*TWO_PI)/inc_t;	//400 = 100 * (1000/500) * 2(int sur voie A seulement)
				int ent = W_rs / 100;
				int dec = abs(W_rs - 100*ent);
				afficher("W(r/s)= %d,%d",ent,dec);
				break; }
			case 2:{
				int N_ds = (1440.0*nbInc)/inc_t;		//1440 = 360° * (1000/500) * 2(int sur voie A seulement)
				afficher("N(d/s)= %d",N_ds);
				break; }
			case 3:{
				int N_is = nbInc*4;						//4 = (1000/500) * 2(int sur voie A seulement)
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
	myEnc.write(0);
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

