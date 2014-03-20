/*
 * type_ia.c
 *
 *  Created on: 20 févr. 2014
 *      Author: seb
 */

#include "millis.h"
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

#include "obj_types.h"

sPt_t _current_pos;
long _start_time;
long last_time=0;
long last_time2=0;
sPath_t path= {.dist = 0.,  .path = NULL };

Obj_arbre arbre[4];
Obj_bac bac;
Obj_feu feu[16];

uint8_t obs_updated[N] = {0};

sObj_t listObj[NB_OBJ] = {
	{.type = E_ARBRE, .numObj=0, .nbObs=1, .numObs[0]=START_ARBRE  ,																 .dist=0, .active=1, .nbEP=2, .entryPoint[0]={{16. , 97.}, 2.,90. }, .entryPoint[1]={{16. , 43.}, 2.,270.}},
	{.type = E_ARBRE, .numObj=1, .nbObs=1, .numObs[0]=START_ARBRE+1, 																 .dist=0, .active=1, .nbEP=2, .entryPoint[0]={{43. , 16.}, 2.,180.}, .entryPoint[1]={{97. , 16.}, 2.,0.  }},
	{.type = E_ARBRE, .numObj=2, .nbObs=1, .numObs[0]=START_ARBRE+2, 																 .dist=0, .active=1, .nbEP=2, .entryPoint[0]={{203., 16.}, 2.,180.}, .entryPoint[1]={{257., 16.}, 2.,0.  }},
	{.type = E_ARBRE, .numObj=3, .nbObs=1, .numObs[0]=START_ARBRE+3, 																 .dist=0, .active=1, .nbEP=2, .entryPoint[0]={{284., 43.}, 2.,270.}, .entryPoint[1]={{284., 97.}, 2.,90. }},
	{.type = E_BAC,   .numObj=0, .nbObs=0, 															     				  			 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{230.,155.}, 2.,90. }},
	{.type = E_BAC,   .numObj=1, .nbObs=0, 																 							 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{70. ,155.}, 2.,90. }},
	{.type = E_FEU,   .numObj=0, .nbObs=1, .numObs[0]=START_FEU    ,																 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{40. , 90.}, 0.,0.  }},
	{.type = E_FEU,   .numObj=1, .nbObs=1, .numObs[0]=START_FEU+1  ,																 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{90. , 40.}, 0.,0.  }},
	{.type = E_FEU,   .numObj=2, .nbObs=1, .numObs[0]=START_FEU+2  , 																 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{90. ,140.}, 0.,0.  }},
	{.type = E_FEU,   .numObj=3, .nbObs=1, .numObs[0]=START_FEU+3  , 																 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{210., 40.}, 0.,0.  }},
	{.type = E_FEU,   .numObj=4, .nbObs=1, .numObs[0]=START_FEU+4  ,																 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{210.,140.}, 0.,0.  }},
	{.type = E_FEU,   .numObj=5, .nbObs=1, .numObs[0]=START_FEU+5  ,																 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{260., 90.}, 0.,0.  }},
	{.type = E_FEU,   .numObj=6, .nbObs=3, .numObs[0]=START_FEU+6  , .numObs[1]=START_TORCHE_FIXE  , .numObs[2]=START_TORCHE_FIXE+1, .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{1.  ,120.}, 0.,0.  }},
	{.type = E_FEU,   .numObj=7, .nbObs=3, .numObs[0]=START_FEU+7  , .numObs[1]=START_TORCHE_FIXE+2, .numObs[2]=START_TORCHE_FIXE+3, .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{130.,  1.}, 0.,0.  }},
	{.type = E_FEU,   .numObj=8, .nbObs=3, .numObs[0]=START_FEU+8  , .numObs[1]=START_TORCHE_FIXE+4, .numObs[2]=START_TORCHE_FIXE+5, .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{170.,  1.}, 0.,0.  }},
	{.type = E_FEU,   .numObj=9, .nbObs=3, .numObs[0]=START_FEU+9  , .numObs[1]=START_TORCHE_FIXE+6, .numObs[2]=START_TORCHE_FIXE+7, .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{299.,120.}, 0.,0.  }},
	};


sNum_t ratio_arbre(void)
    {
    //décroissance lineaire
    float res;
    res=-(millis()-_start_time)/1000+90;
    return res;
    }

sNum_t ratio_bac(void)
    {
    //Croissance lineaire
    float res;
    res=(millis()-_start_time)/1000-20;
    return res;
    }

void printListObj(void)
    {
    int i,j;
    printf("ListObj :\n");
    for(i=0 ; i<NB_OBJ ; i++)
        {
        printf("type=%d, ",listObj[i].type);
        printf("numObj=%d, ",listObj[i].numObj);
        printf("nbObs=%d, ",listObj[i].nbObs);
        printf("numObs={");
        for(j=0 ; j<listObj[i].nbObs ; j++) printf("%d, ",listObj[i].numObs[j]);
        printf("}, ");
        printf("dist=%f, ",listObj[i].dist);
        printf("active=%d, ",listObj[i].active);
        printf("nbEP=%d, ",listObj[i].nbEP);
        printf("entryPoint={");
        for(j=0 ; j<listObj[i].nbEP ; j++)
        	{
        	printf("{{%d, %d}, ",(int)listObj[i].entryPoint[j].c.x,(int)listObj[i].entryPoint[j].c.y);
        	printf("%d, ",(int)listObj[i].entryPoint[j].radiusEP);
        	printf("%d}, ",listObj[i].entryPoint[j].angleEP);
        	}
        printf("}, ");
        printf("\n");
        }
    printf("\n");
    }

void printObsActive(void)
	{
	int i;
	printf("Liste des obs[i].active :\n");
	for(i=0 ; i<N ; i++)
		printf("obs[%d].active=%d\n",i,obs[i].active);
	printf("\n");
	}
/*
int sgnFire(uint8_t num, sNum_t theta)
	{
	if(theta<0 || theta>120)
		{
		printf("Error in sgnFire theta<0 or theta>120 degre\n");
		return(0);
		}
	switch(num)
		{
		case(1) :
			if(theta<90) return(1);
			else return(-1);
			break;
		case(2) :
			return(-1);
			break;
		case(3) :
			if(theta<30) return(-1);
		}

	}
*/
void createEPfire(sPt_t *pt, sNum_t theta, sNum_t r, int numObj)
	{
	int i;
	printf("ent x=%f y%f\n", pt->x, pt->y);
	for(i=0 ; i <3 ; i++)
		{
		listObj[numObj].entryPoint[i].c.x=r*cos(theta*M_PI/180.+i*2.*M_PI/3.)+pt->x;
		listObj[numObj].entryPoint[i].c.y=r*sin(theta*M_PI/180.+i*2.*M_PI/3.)+pt->y;
		printf("x=%f y=%f\n",r*cos(theta*M_PI/180.+i*2.*M_PI/3.)+pt->x,r*sin(theta*M_PI/180.+i*2.*M_PI/3.)+pt->y);
		}
	}

void init_ele(void)
    {
    printf("Debut de l'initialisation des elements du jeu\n");
    int i, j;
    sPt_t pt;

    //Initialisation des arbres
    for(i=0 ; i<4 ; i++)
    	{
    	listObj[i].typeStruct = &arbre[listObj[i].numObj];
    	((Obj_arbre*)listObj[i].typeStruct)->nb_point=10;                      //1 fruit pouri par arbre
    	for(j=0 ; j<6 ; j++) ((Obj_arbre*)listObj[i].typeStruct)->eFruit[j]=0; //par défaut tous les fruit sont bon
    	}

    //Initialisation des bacs
    for(i=4 ; i<6 ; i++)
    	{
    	listObj[i].typeStruct = &bac;
        ((Obj_bac*)listObj[i].typeStruct)->nb_point=0;
    	}
    if(COLOR==1)listObj[4].active=0;
    else listObj[5].active=0;


    //Initialisation des feux
    for(i=6 ; i<16 ; i++)
        {
    	listObj[i].typeStruct = &feu[listObj[i].numObj];
        ((Obj_feu*)listObj[i].typeStruct)->pos=0;
        ((Obj_feu*)listObj[i].typeStruct)->nb_point=2;
        listObj[i].nbEP=3;
        pt.x=listObj[i].entryPoint[0].c.x;
        pt.y=listObj[i].entryPoint[0].c.y;
        createEPfire(&pt, 0 , R_ROBOT+8, i);
        }

    printListObj();
    printf("Fin de l'initialisation des elements du jeu\n");
    }



void send_robot(sPath_t path)
    {
    sMsg outMsg;
    int i, ret ;
    static int tid = 0;
    tid++;
    if(path.path)
        for(i = 0; i < path.path_len; i++) {
            printf("  %u: p1 x%f y%f, p2 x%f y%f, obs x%f y%f r%.2f, a_l%f s_l%f\n", i, path.path[i].p1.x, path.path[i].p1.y, path.path[i].p2.x, path.path[i].p2.y,path.path[i].obs.c.x,path.path[i].obs.c.y, path.path[i].obs.r,path.path[i].arc_len,path.path[i].seg_len);

            outMsg.header.type = E_TRAJ;
            outMsg.header.size = sizeof(outMsg.payload.traj);

            outMsg.payload.traj.p1_x = path.path[i].p1.x;
            outMsg.payload.traj.p1_y = path.path[i].p1.y;
            outMsg.payload.traj.p2_x = path.path[i].p2.x;
            outMsg.payload.traj.p2_y = path.path[i].p2.y;
            outMsg.payload.traj.seg_len = path.path[i].seg_len;

            outMsg.payload.traj.c_x = path.path[i].obs.c.x;
            outMsg.payload.traj.c_y = path.path[i].obs.c.y;
            outMsg.payload.traj.c_r = path.path[i].obs.r;
            outMsg.payload.traj.arc_len = path.path[i].arc_len;

            outMsg.payload.traj.sid = i;
            outMsg.payload.traj.tid = tid;

            ret = role_send(&outMsg);
            if(ret < 0) printf("role_send(E_TRAJ) failed #%i\n", -ret);

            usleep(1000);
        }
    }

int get_position( sPt_t *pos)
    {
    if(obs[0].moved)
        {
        *pos = obs[0].c;
        return 1;
        }
    else
        {
        return 0;
        }
    }

float sign(float x) //retourne -1 ou 1, 0 si nul
    {
    if(x==0) return 0;
    return x/fabs(x);
    }

void project_point(sNum_t xp, sNum_t yp, sNum_t rc, sNum_t xc, sNum_t yc, sPt_t *point)
	{
	sPt_t o_c= {xc ,yc};
	sPt_t o_p= {xp ,yp};
	sVec_t v;
	sNum_t r=rc, n, d;

	convPts2Vec(&o_c, &o_p, &v);
	normVec(&v, &n);

	d = n - fabs(r);

	if(fabs(d) > 2.) printf("!!! far from the circle (%.2fcm)...\n", d);

	point->x = o_c.x + v.x*(fabs(r) + 0.1)/n;
	point->y = o_c.y + v.y*(fabs(r) + 0.1)/n;
	}
