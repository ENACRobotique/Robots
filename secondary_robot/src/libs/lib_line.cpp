#include "lib_us.h"
#include "lib_intensity_cny70.h"
#include "lib_line.h"
#include "lib_move.h"

#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

//uint8_t decodeLines(uint8_t *c /* capteurs */, uint8_t *n /* nombre , uint8_t *p, uint8_t *d) {

//uint8_t getLines(uint8_t *c,


/*void asserLine()
	{
	uint8_t tab[8];
	//Serial.println("asserLine");
	//Détection de la ligne noir ou non par les capteurs
	int i=0;
	for(i=0;i<8;i++)
		{
		if(getIntensity(i)>4500) //ligne noir
			{
			tab[i]=1;
			}
		else //autre couleur
			{
			tab[i]=0;
			}
		}*/

	/*//Autre methode en cour
	int nbLine=0;

for(int i=0;i<8;i++)
	{
	if(tab[i]==1); //line détection

	}
	 */

	//Proportion gauche/droite
	/*int Sgauche=tab[4]+tab[5]+tab[6]+tab[7];
	int Sdroite=tab[0]+tab[1]+tab[2]+tab[3];
	int diff=Sdroite-Sgauche;

	#ifdef DEBUG
		Serial.println(diff);
	#endif

		if(diff!=0) //Garde en mémoire si plus de ligne ou au milieu
			{
			move(70,diff*10); //Attention max 250
			}
	}*/

void asserLine()
	{
	uint8_t tab[8];
	int i=0;
	for(i=0;i<8;i++)
		{
		if(getIntensity(i)>4500) //ligne noir
			{
			tab[i]=1;
			}
		else //autre couleur
			{
			tab[i]=0;
			}
		}

	//Gauche/Droite
	int capG =(tab[4])+(tab[5])+(tab[6])+(tab[7]*10);
	int capD =(tab[3])+(tab[2])+(tab[1])+(tab[0]*10);
	int deg=0;

	/*

	deg=(capG*(-6));
	move(80,CLAMP(-30,deg,30));

	*/

	//int degD =tab[1]+tab[2]+tab[3]+tab[0];

//	int prv=0;

	if(capG==0 && capD==0)
		{
			move(60,8);
		}

	if((tab[7]+tab[6])==0 && (tab[0]+tab[1])==0 && tab[2]+tab[3]+tab[4]+tab[5]!=0)
	{
		move(60,8);
	}
	/*else{
	if((tab[7])!=0 && (tab[0]+tab[1])==0 && tab[4]+tab[5]==0)
		{
			deg=(capG*(1));
			move(80,deg);

		}*/

	else{

			if(capG>capD){
				deg=(capG*(-5));
				move(60,deg);

				//deg=(capG*(-8));
				//move(-80,deg);
//				prv=1;
				Serial.println("un peu G");
			}

			if(capG<capD){
				deg=(capD*(5));
				move(60,deg+8);
				Serial.println("un peu D");
				//deg=(capD*(8));
				//move(-80,deg);
//				prv=2;
			}

	}

/*
	if(prv==1){
		move(80,-40);
		//prv=0;
		Serial.println("trop G");
	}
	if(prv==2){
		move(80,40);
		//prv=0;
		Serial.println("trop D");

	}
*/

}


int lineDetection(void)
	{
	int i;
	int nb=0;
	for(i=0;i<8;i++)
		{
		if(getIntensity(i)>4500)nb++;
		}
	if(nb>1) return 1; //Si 2 capteurs détecte la ligne
	return 0;
	}
