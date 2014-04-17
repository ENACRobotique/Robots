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
#include "obj_fct.h"

#include "obj_types.h"

sPt_t _current_pos = {0., 0.};
long _start_time;
long last_time=0;
long last_time2=0;
sPath_t path= {.dist = 0.,  .path = NULL };

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

uint8_t obs_updated[N] = {0};

sObj_t listObj[NB_OBJ] = {
	{.type = E_ARBRE, .numObj=0, .nbObs=1, .numObs[0]=START_ARBRE  ,																 .dist=0, .active=1, .nbEP=2, .entryPoint[0]={{16. , 97.}, 10.,90. }, .entryPoint[1]={{16. , 45.}, 10.,270.}},
	{.type = E_ARBRE, .numObj=1, .nbObs=1, .numObs[0]=START_ARBRE+1, 																 .dist=0, .active=1, .nbEP=2, .entryPoint[0]={{43. , 16.}, 10.,180.}, .entryPoint[1]={{97. , 16.}, 10.,0.  }},
	{.type = E_ARBRE, .numObj=2, .nbObs=1, .numObs[0]=START_ARBRE+2, 																 .dist=0, .active=1, .nbEP=2, .entryPoint[0]={{203., 16.}, 10.,180.}, .entryPoint[1]={{257., 16.}, 10.,0.  }},
	{.type = E_ARBRE, .numObj=3, .nbObs=1, .numObs[0]=START_ARBRE+3, 																 .dist=0, .active=1, .nbEP=2, .entryPoint[0]={{284., 43.}, 10.,270.}, .entryPoint[1]={{284., 97.}, 10.,90. }},
	{.type = E_BAC,   .numObj=0, .nbObs=0, 															     				  			 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{230.,155.}, 10.,270.}},
	{.type = E_BAC,   .numObj=1, .nbObs=0, 																 							 .dist=0, .active=1, .nbEP=1, .entryPoint[0]={{70. ,155.}, 10.,270.}},
	{.type = E_FEU,   .numObj=0, .nbObs=1, .numObs[0]=START_FEU    ,																 .dist=0, .active=1, .nbEP=1},
	{.type = E_FEU,   .numObj=1, .nbObs=1, .numObs[0]=START_FEU+1  ,																 .dist=0, .active=1, .nbEP=1},
	{.type = E_FEU,   .numObj=2, .nbObs=1, .numObs[0]=START_FEU+2  , 																 .dist=0, .active=1, .nbEP=1},
	{.type = E_FEU,   .numObj=3, .nbObs=1, .numObs[0]=START_FEU+3  , 																 .dist=0, .active=1, .nbEP=1},
	{.type = E_FEU,   .numObj=4, .nbObs=1, .numObs[0]=START_FEU+4  ,																 .dist=0, .active=1, .nbEP=1},
	{.type = E_FEU,   .numObj=5, .nbObs=1, .numObs[0]=START_FEU+5  ,																 .dist=0, .active=1, .nbEP=1},
	{.type = E_FEU,   .numObj=6, .nbObs=3, .numObs[0]=START_FEU+6  , .numObs[1]=START_TORCHE_FIXE  , .numObs[2]=START_TORCHE_FIXE+1, .dist=0, .active=1, .nbEP=1},
	{.type = E_FEU,   .numObj=7, .nbObs=3, .numObs[0]=START_FEU+7  , .numObs[1]=START_TORCHE_FIXE+2, .numObs[2]=START_TORCHE_FIXE+3, .dist=0, .active=1, .nbEP=1},
	{.type = E_FEU,   .numObj=8, .nbObs=3, .numObs[0]=START_FEU+8  , .numObs[1]=START_TORCHE_FIXE+4, .numObs[2]=START_TORCHE_FIXE+5, .dist=0, .active=1, .nbEP=1},
	{.type = E_FEU,   .numObj=9, .nbObs=3, .numObs[0]=START_FEU+9  , .numObs[1]=START_TORCHE_FIXE+6, .numObs[2]=START_TORCHE_FIXE+7, .dist=0, .active=1, .nbEP=1},
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

void printListObj(void){
    int i,j;
    printf("ListObj :\n");
    for(i=0 ; i<NB_OBJ ; i++){
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
        for(j=0 ; j<listObj[i].nbEP ; j++){
        	printf("{{%d, %d}, ",(int)listObj[i].entryPoint[j].c.x,(int)listObj[i].entryPoint[j].c.y);
        	printf("%d, ",(int)listObj[i].entryPoint[j].radiusEP);
        	printf("%d}, ",listObj[i].entryPoint[j].angleEP);
        	}
        printf("}, ");
        printf("\n");
        }
    printf("\n");
    }

void printObsActive(void){
	int i;
	printf("Liste des obs[i].active :\n");
	for(i=0 ; i<N ; i++)
		printf("obs[%d].active=%d\n",i,obs[i].active);
	printf("\n");
	}

void createEPfire(sPt_t *pt, sNum_t theta, sNum_t r, int numObj){ //TODO determiner angle d'acces
	int i;
	printf("ent x=%f y%f\n", pt->x, pt->y);
	for(i=0 ; i <3 ; i++){
		listObj[numObj].entryPoint[i].c.x=r*cos(theta*M_PI/180.+i*2.*M_PI/3.)+pt->x;
		listObj[numObj].entryPoint[i].c.y=r*sin(theta*M_PI/180.+i*2.*M_PI/3.)+pt->y;
		listObj[numObj].entryPoint[i].angleEP=theta+i*120;
		listObj[numObj].entryPoint[i].radiusEP=FIRE_RADIUS_EP;
		printf("x=%f y=%f\n",r*cos(theta*M_PI/180.+i*2.*M_PI/3.)+pt->x,r*sin(theta*M_PI/180.+i*2.*M_PI/3.)+pt->y);
		}
	}

void createEPfire2(int numObj){
	int i;
	printf("numObj=%d et pos=%d\n",numObj,feu[numObj-6].pos);
	switch(feu[numObj-6].pos){
		case 1 :
		case 2 :
			listObj[numObj].nbEP=3;
			for(i=0;i<3;i++){
				listObj[numObj].entryPoint[i].angleEP=feu[numObj-6].angle+120*i;
				listObj[numObj].entryPoint[i].c.x=(R_ROBOT+8)*cos(listObj[numObj].entryPoint[i].angleEP*M_PI/180.)+feu[numObj-6].c.x;
				listObj[numObj].entryPoint[i].c.y=(R_ROBOT+8)*sin(listObj[numObj].entryPoint[i].angleEP*M_PI/180.)+feu[numObj-6].c.y;
				listObj[numObj].entryPoint[i].radiusEP=FIRE_RADIUS_EP;
				}
			break;
		case 3 :
			listObj[numObj].nbEP=1;
			if(COLOR==1)i=0;
			else i=1;

			if((feu[numObj-6].angle>=180) && (i==1)) listObj[numObj].entryPoint[0].angleEP=feu[numObj-6].angle-180;
			else listObj[numObj].entryPoint[0].angleEP=feu[numObj-6].angle+180*i;
			listObj[numObj].entryPoint[0].c.x=(R_ROBOT+8)*cos(listObj[numObj].entryPoint[0].angleEP*M_PI/180.)+feu[numObj-6].c.x;
			listObj[numObj].entryPoint[0].c.y=(R_ROBOT+8)*sin(listObj[numObj].entryPoint[0].angleEP*M_PI/180.)+feu[numObj-6].c.y;
			listObj[numObj].entryPoint[0].radiusEP=FIRE_RADIUS_EP;

			break;
		case 4 :
			listObj[numObj].nbEP=1;
			listObj[numObj].entryPoint[0].radiusEP=FIRE_RADIUS_EP;
			if(feu[numObj-6].c.x==1.){
				listObj[numObj].entryPoint[0].c.x=20.;
				listObj[numObj].entryPoint[0].c.y=120.;
				listObj[numObj].entryPoint[0].angleEP=0.;
				}
			if(feu[numObj-6].c.x==130.){
				listObj[numObj].entryPoint[0].c.x=130.;
				listObj[numObj].entryPoint[0].c.y=20.;
				listObj[numObj].entryPoint[0].angleEP=90.;
				}
			if(feu[numObj-6].c.x==170.){
				listObj[numObj].entryPoint[0].c.x=170.;
				listObj[numObj].entryPoint[0].c.y=20.;
				listObj[numObj].entryPoint[0].angleEP=90.;
				}
			if(feu[numObj-6].c.x==299.){
				listObj[numObj].entryPoint[0].c.x=280.;
				listObj[numObj].entryPoint[0].c.y=120.;
				listObj[numObj].entryPoint[0].angleEP=180.;
				}
			break;
		default :
			printf("Errorin switch createEPfire\n");
			getchar();
			break;
		}
	}


void TransElTraj(sTrajEl_t *traj, int x, int y){
	//axis x
	traj->p1.x += x;
	traj->p2.x += x;
	traj->obs.c.x += x;

	//axis y
	traj->p1.y += y;
	traj->p2.y += y;
	traj->obs.c.y += y;
	}

void SymElTraj(sTrajEl_t *traj, int x, int y){// x!=0 symmetry in x in zero and idem for y
	if(y!=0){
		traj->p1.x*=-1;
		traj->p2.x*=-1;
		traj->obs.c.x*=-1;
		traj->obs.r*=-1;
		}

	if(x!=0){
		traj->p1.y=-traj->p1.y;
		traj->p2.y=-traj->p2.y;
		traj->obs.c.y=-traj->obs.c.y;
		traj->obs.r=-traj->obs.r;
		}
	}

void Rot90Traj(sTrajEl_t *traj){
	float tempA, tempB, tempC;

	tempA=traj->p1.x;
	tempB=traj->p2.x;
	tempC=traj->obs.c.x;

	traj->p1.x = -traj->p1.y;
	traj->p2.x = -traj->p2.y;
	traj->obs.c.x = -traj->obs.c.y;

	traj->p1.y = tempA;
	traj->p2.y = tempB;
	traj->obs.c.y = tempC;
	}

sNum_t arc_len2(sPt_t *p2_1, sPt_t *oc, sNum_t or, sPt_t *p2_3){
    sVec_t v1, v3;
    sNum_t d, c;

    if(fabs(or) < LOW_THR)
        return 0.;

    convPts2Vec(oc, p2_1, &v1);
    convPts2Vec(oc, p2_3, &v3);

    dotVecs(&v1, &v3, &d);
    crossVecs(&v1, &v3, &c);

    d = d/(or*or);
    // d must be between -1 and 1 but because we do not use the true length of v1 and v3
    // (we use r instead to avoid some heavy calculations) it may be a little outside of this interval
    // so, let's just be sure we stay in this interval for acos to give a result
    if(d > 1.) {
        d = 1.;
    }
    else if(d < -1.) {
        d = -1.;
    }

    d = acos(d);

    if(or > 0.) {  // clock wise
        if(c > 0) {
            d = 2*M_PI - d;
        }
    }
    else {  // counter clock wise
        if(c < 0) {
            d = 2*M_PI - d;
        }
    }

    return fabs(d*or);
}

void init_ele(void){
    printf("Debut de l'initialisation des elements du jeu\n");
    int i, j;

    //Initialisation des arbres
    for(i=0 ; i<4 ; i++){
    	listObj[i].typeStruct = &arbre[listObj[i].numObj];
    	((Obj_arbre*)listObj[i].typeStruct)->nb_point=10;                      //1 fruit pouri par arbre
    	for(j=0 ; j<6 ; j++) ((Obj_arbre*)listObj[i].typeStruct)->eFruit[j]=0; //par défaut tous les fruit sont bon
    	}
    //Add Entry Point in struct arbre
    for(j=0 ; j<4 ; j++){
    	arbre[j].EntryPoint1=listObj[j].entryPoint[0].c;
    	arbre[j].EntryPoint2=listObj[j].entryPoint[1].c;
    	}

    //Initialisation des bacs
    for(i=4 ; i<6 ; i++){
    	listObj[i].typeStruct = &bac;
        ((Obj_bac*)listObj[i].typeStruct)->nb_point=0;
    	}
    if(COLOR==1)listObj[4].active=0;
    else listObj[5].active=0;


    //Initialisation des feux
    for(i=6 ; i<16 ; i++){
    	listObj[i].typeStruct = &feu[listObj[i].numObj];
        ((Obj_feu*)listObj[i].typeStruct)->nb_point=2;
        listObj[i].nbEP=3;
        createEPfire2(i);
        }

	#if DEBUG
		printListObj();
	#endif
    printf("Fin de l'initialisation des elements du jeu\n");
    }



void send_robot(sPath_t path){
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

int get_position( sPt_t *pos){
    if(obs[0].moved){
        *pos = obs[0].c;
        return 1;
        }
    else{
        return 0;
        }
    }

float sign(float x){ //retourne -1 ou 1, 0 si nul
    if(x==0) return 0;
    return x/fabs(x);
    }

void project_point(sNum_t xp, sNum_t yp, sNum_t rc, sNum_t xc, sNum_t yc, sPt_t *point){
	sPt_t o_c= {xc ,yc};
	sPt_t o_p= {xp ,yp};
	sVec_t v;
	sNum_t r=rc, n, d;

	convPts2Vec(&o_c, &o_p, &v);
	normVec(&v, &n);

	d = n - fabs(r);

	if(fabs(d) > 2.) printf("!!! far from the circle (%.2fcm)...\n", d);

	point->x = o_c.x + v.x*(fabs(r) + 0.1)/n+0.1*sign(v.x*(fabs(r) + 0.1)/n);
	point->y = o_c.y + v.y*(fabs(r) + 0.1)/n+0.1*sign(v.y*(fabs(r) + 0.1)/n);
	}
