#include "lib_us.h"
#include "lib_intensity_cny70.h"
#include "lib_line.h"
#include "lib_move.h"


//uint8_t decodeLines(uint8_t *c /* capteurs */, uint8_t *n /* nombre , uint8_t *p, uint8_t *d) {

//uint8_t getLines(uint8_t *c,


void asserLine()
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
		}

	/*//Autre methode en cour
	int nbLine=0;

for(int i=0;i<8;i++)
	{
	if(tab[i]==1); //line détection

	}
	 */

	//Proportion gauche/droite
	int Sgauche=tab[4]+tab[5]+tab[6]+tab[7];
	int Sdroite=tab[0]+tab[1]+tab[2]+tab[3];
	int diff=Sdroite-Sgauche;

	#ifdef DEBUG
		Serial.println(diff);
	#endif

		if(diff!=0) //Garde en mémoire si plus de ligne ou au milieu
			{
			move(100,diff*10); //Attention max 250
			}
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
