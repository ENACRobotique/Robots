/*
 * type_ia.c
 *
 *  Created on: 20 févr. 2014
 *      Author: seb
 */

#include"type_ia.h"

sObs_t _current_pos;
unsigned long _start_time;
sPath_t path= {.dist = 0.,  .path = NULL };

sObj_t list_obj[]={
	{E_ARBRE, 0., 1, 2, 1 , &arbre[0]},
	{E_ARBRE, 1., 2, 2, 1 , &arbre[1]},
	{E_ARBRE, 2., 3, 2, 1 , &arbre[2]},
	{E_ARBRE, 3., 4, 2, 1 , &arbre[3]},
	{E_BAC,   0., 5 , 1, 1 , &bac}
	};

sObs_t obs_PA[]={
{{10. , 90.}, 0, 1,1},
{{10. , 50.}, 0 ,1,1},
{{50. , 10.}, 0 ,1,1},
{{90. , 10.}, 0 ,1,1},
{{210. ,10.}, 0 ,1,1},
{{250. ,10.}, 0 ,1,1},
{{290. ,50.}, 0 ,1,1},
{{290. ,90.}, 0 ,1,1},
{{225.,170.},0,1,1} //bac rouge
};


unsigned long millis() //retourne le temps depuis le depart de _start_time en ms
	{
    unsigned long res = (time(NULL) - _start_time)*1000;
    return res;
	}

sNum_t ratio_arbre(void)
	{
	//décroissance lineaire
	int res;
	res=-millis()/1000+90;
	return res;
	}

sNum_t ratio_bac(void)
	{
	//Croissance lineaire
	int res;
	res=millis()/1000-20;
	return res;
	}

