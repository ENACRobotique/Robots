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
eColor_t color = 1 ;//0=red and 1=yellow
int current_obj=-1;

sPath_t path= {.dist = 0.,  .path = NULL };
uint8_t obs_updated[N] = {0};
sObjBasket basket;

sObj_t listObj[NB_OBJ] = {
	{.etype = E_ARBRE, .state = ACTIVE, .numObj=0, .nbObs=1, .numObs[0]=START_ARBRE  ,																   .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0].angleEP = 90 , .entryPoint[1].angleEP = 270, .utype.tree.x = 0  , .utype.tree.y = 70, .utype.tree.rot=0, .utype.tree.angle = 90. },
	{.etype = E_ARBRE, .state = ACTIVE, .numObj=1, .nbObs=1, .numObs[0]=START_ARBRE+1, 																   .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0].angleEP = 180, .entryPoint[1].angleEP = 0  , .utype.tree.x = 70 , .utype.tree.y = 0 , .utype.tree.rot=1, .utype.tree.angle = 180.},
	{.etype = E_ARBRE, .state = ACTIVE, .numObj=2, .nbObs=1, .numObs[0]=START_ARBRE+2, 																   .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0].angleEP = 180, .entryPoint[1].angleEP = 0  , .utype.tree.x = 230, .utype.tree.y = 0 , .utype.tree.rot=1, .utype.tree.angle = 180.},
	{.etype = E_ARBRE, .state = ACTIVE, .numObj=3, .nbObs=1, .numObs[0]=START_ARBRE+3, 																   .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0].angleEP = 270, .entryPoint[1].angleEP = 90 , .utype.tree.x = 300, .utype.tree.y = 70, .utype.tree.rot=0, .utype.tree.angle = 270.},
	{.etype = E_BAC,   .state = ACTIVE, .numObj=0, .nbObs=0, 															     				  		   .dist=0, .active=1, .done=0, .nbEP=3, .entryPoint[0]={{215.,155.}, 10.,270.}, .entryPoint[1]={{225.,155.}, 10.,270.}, .entryPoint[2]={{235.,155.}, 10.,270.}},
	{.etype = E_BAC,   .state = ACTIVE, .numObj=1, .nbObs=0, 																 						   .dist=0, .active=1, .done=0, .nbEP=3, .entryPoint[0]={{65. ,155.}, 10.,270.}, .entryPoint[1]={{75. ,155.}, 10.,270.}, .entryPoint[2]={{85. ,155.}, 10.,270.}},
	{.etype = E_FEU,   .state = ACTIVE, .numObj=0, .nbObs=1, .numObs[0]=START_FEU    ,																   .dist=0, .active=1, .done=0, .nbEP=1, .utype.fire.c={40. , 90.},.utype.fire.pos=3, .utype.fire.angle=270.},
	{.etype = E_FEU,   .state = ACTIVE, .numObj=1, .nbObs=1, .numObs[0]=START_FEU+1  ,																   .dist=0, .active=1, .done=0, .nbEP=1, .utype.fire.c={90. , 40.},.utype.fire.pos=3, .utype.fire.angle=180.},
	{.etype = E_FEU,   .state = ACTIVE, .numObj=2, .nbObs=1, .numObs[0]=START_FEU+2  , 															       .dist=0, .active=1, .done=0, .nbEP=1, .utype.fire.c={90. ,140.},.utype.fire.pos=3, .utype.fire.angle=0.  },
	{.etype = E_FEU,   .state = ACTIVE, .numObj=3, .nbObs=1, .numObs[0]=START_FEU+3  , 																   .dist=0, .active=1, .done=0, .nbEP=1, .utype.fire.c={210., 40.},.utype.fire.pos=3, .utype.fire.angle=180.},
	{.etype = E_FEU,   .state = ACTIVE, .numObj=4, .nbObs=1, .numObs[0]=START_FEU+4  ,															       .dist=0, .active=1, .done=0, .nbEP=1, .utype.fire.c={210.,140.},.utype.fire.pos=3, .utype.fire.angle=0.  },
	{.etype = E_FEU,   .state = ACTIVE, .numObj=5, .nbObs=1, .numObs[0]=START_FEU+5  ,																   .dist=0, .active=1, .done=0, .nbEP=1, .utype.fire.c={260., 90.},.utype.fire.pos=3, .utype.fire.angle=90. },
	{.etype = E_FEU,   .state = ACTIVE, .numObj=6, .nbObs=3, .numObs[0]=START_FEU+6  , .numObs[1]=START_TORCHE_FIXE  , .numObs[2]=START_TORCHE_FIXE+1, .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0]={{22. ,80.}, 10.,90. }, .entryPoint[0]={{22. ,80.}, 10.,270.}, .utype.fire.c={1.  ,120.},.utype.fire.pos=4, .utype.fire.angle=0.  },
	{.etype = E_FEU,   .state = ACTIVE, .numObj=7, .nbObs=3, .numObs[0]=START_FEU+7  , .numObs[1]=START_TORCHE_FIXE+2, .numObs[2]=START_TORCHE_FIXE+3, .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0]={{130.,22.}, 10.,180.}, .entryPoint[0]={{130.,22.}, 10.,0.  }, .utype.fire.c={130.,  1.},.utype.fire.pos=4, .utype.fire.angle=270.},
	{.etype = E_FEU,   .state = ACTIVE, .numObj=8, .nbObs=3, .numObs[0]=START_FEU+8  , .numObs[1]=START_TORCHE_FIXE+4, .numObs[2]=START_TORCHE_FIXE+5, .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0]={{170.,22.}, 10.,180.}, .entryPoint[0]={{170.,22.}, 10.,0.  }, .utype.fire.c={170.,  1.},.utype.fire.pos=4, .utype.fire.angle=90. },
	{.etype = E_FEU,   .state = ACTIVE, .numObj=9, .nbObs=3, .numObs[0]=START_FEU+9  , .numObs[1]=START_TORCHE_FIXE+6, .numObs[2]=START_TORCHE_FIXE+7, .dist=0, .active=1, .done=0, .nbEP=2, .entryPoint[0]={{278.,80.}, 10.,270.}, .entryPoint[0]={{278.,80.}, 10.,90. }, .utype.fire.c={299.,120.},.utype.fire.pos=4, .utype.fire.angle=0.  },
	};






