/*
 * type_ia.c
 *
 *  Created on: 20 févr. 2014
 *      Author: seb
 */

#include "obj_types.h"



long _start_time;
long last_time=0;

sPt_t pt_select;
sPt_t _current_pos = {0., 0.};

int mode_obj=0;
sNum_t speed=0;
sNum_t theta_robot=0.;
int starting_cord = 0;
int mode_switch = 0;
int color = 0 ;//0=red and 1=yellow

sPath_t path= {.dist = 0.,  .path = NULL };

uint8_t obs_updated[N] = {0};

Obj_arbre arbre[4]={
	{.x=0  ,.y=70,.rot=0},
	{.x=70 ,.y=0 ,.rot=1},
	{.x=230,.y=0 ,.rot=1},
	{.x=300,.y=70,.rot=0}
	};

Obj_bac bac;

Obj_feu feu[16]={
	{.c={40. , 90.},.pos=3, .angle=270.},
	{.c={90. , 40.},.pos=3, .angle=180.},
	{.c={90. ,140.},.pos=3, .angle=0.},
	{.c={210., 40.},.pos=3, .angle=180.},
	{.c={210.,140.},.pos=3, .angle=0.},
	{.c={260., 90.},.pos=3, .angle=90.},

	{.c={1.  ,120.},.pos=4, .angle=0.},
	{.c={130.,  1.},.pos=4, .angle=270.},
	{.c={170.,  1.},.pos=4, .angle=90.},
	{.c={299.,120.},.pos=4, .angle=0.},

	{.c={90.,90.},.pos=1, .angle=90},
	{.c={90.,90.},.pos=1, .angle=90},
	{.c={90.,90.},.pos=2, .angle=90},
	{.c={210.,90.},.pos=2, .angle=90},
	{.c={210.,90.},.pos=2, .angle=90},
	{.c={210.,90.},.pos=1, .angle=90},
	};


sObj_t listObj[NB_OBJ] = {
	{.type = E_ARBRE, .numObj=0, .nbObs=1, .numObs[0]=START_ARBRE  ,																 .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0]={{16. , 97.}, 10.,90. }, .entryPoint[1]={{16. , 45.}, 10.,270.}},
	{.type = E_ARBRE, .numObj=1, .nbObs=1, .numObs[0]=START_ARBRE+1, 																 .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0]={{43. , 16.}, 10.,180.}, .entryPoint[1]={{97. , 16.}, 10.,0.  }},
	{.type = E_ARBRE, .numObj=2, .nbObs=1, .numObs[0]=START_ARBRE+2, 																 .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0]={{203., 16.}, 10.,180.}, .entryPoint[1]={{257., 16.}, 10.,0.  }},
	{.type = E_ARBRE, .numObj=3, .nbObs=1, .numObs[0]=START_ARBRE+3, 																 .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0]={{284., 43.}, 10.,270.}, .entryPoint[1]={{284., 97.}, 10.,90. }},
	{.type = E_BAC,   .numObj=0, .nbObs=0, 															     				  			 .dist=0, .active=1, .done=0, .nbEP=1, .entryPoint[0]={{230.,155.}, 10.,270.}},
	{.type = E_BAC,   .numObj=1, .nbObs=0, 																 							 .dist=0, .active=1, .done=0, .nbEP=1, .entryPoint[0]={{70. ,155.}, 10.,270.}},
	{.type = E_FEU,   .numObj=0, .nbObs=1, .numObs[0]=START_FEU    ,																 .dist=0, .active=1, .done=0, .nbEP=1},
	{.type = E_FEU,   .numObj=1, .nbObs=1, .numObs[0]=START_FEU+1  ,																 .dist=0, .active=1, .done=0, .nbEP=1},
	{.type = E_FEU,   .numObj=2, .nbObs=1, .numObs[0]=START_FEU+2  , 																 .dist=0, .active=1, .done=0, .nbEP=1},
	{.type = E_FEU,   .numObj=3, .nbObs=1, .numObs[0]=START_FEU+3  , 																 .dist=0, .active=1, .done=0, .nbEP=1},
	{.type = E_FEU,   .numObj=4, .nbObs=1, .numObs[0]=START_FEU+4  ,																 .dist=0, .active=1, .done=0, .nbEP=1},
	{.type = E_FEU,   .numObj=5, .nbObs=1, .numObs[0]=START_FEU+5  ,																 .dist=0, .active=1, .done=0, .nbEP=1},
	{.type = E_FEU,   .numObj=6, .nbObs=3, .numObs[0]=START_FEU+6  , .numObs[1]=START_TORCHE_FIXE  , .numObs[2]=START_TORCHE_FIXE+1, .dist=0, .active=0, .done=0, .nbEP=1},
	{.type = E_FEU,   .numObj=7, .nbObs=3, .numObs[0]=START_FEU+7  , .numObs[1]=START_TORCHE_FIXE+2, .numObs[2]=START_TORCHE_FIXE+3, .dist=0, .active=0, .done=0, .nbEP=1},
	{.type = E_FEU,   .numObj=8, .nbObs=3, .numObs[0]=START_FEU+8  , .numObs[1]=START_TORCHE_FIXE+4, .numObs[2]=START_TORCHE_FIXE+5, .dist=0, .active=0, .done=0, .nbEP=1},
	{.type = E_FEU,   .numObj=9, .nbObs=3, .numObs[0]=START_FEU+9  , .numObs[1]=START_TORCHE_FIXE+6, .numObs[2]=START_TORCHE_FIXE+7, .dist=0, .active=0, .done=0, .nbEP=1},
	};






