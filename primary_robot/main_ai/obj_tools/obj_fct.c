/*
 * obj_fct.c
 *
 *  Created on: 29 mars 2014
 *      Author: seb
 */

#include "obj_fct.h"



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


int get_position( sPt_t *pos){
	*pos = obs[0].c;
    obs_updated[0]++;
	return 1;
    }

float sign(float x){ //retourne -1 ou 1, 0 si nul
    if(x==0) return 0;
    return x/fabs(x);
    }

int test_in_obs(void){ //retourne le numéros de l'obstable si la position est a l'interieur de celui ci
    //FIXME si le robot dans plusieurs obstable
    int i;
    for(i=1; i<N-1;i++){
        if(obs[i].active==0)continue;
        if( sqrt((obs[i].c.x-_current_pos.x)*(obs[i].c.x-_current_pos.x)+(obs[i].c.y-_current_pos.y)*(obs[i].c.y-_current_pos.y)) < obs[i].r){
        	printf("Le robot est dans l'obstacle n=%i, robs=%f, xobs=%f, yobs=%f, currentpos x=%f, y=%f\n",i,obs[i].r,obs[i].c.x,obs[i].c.y, _current_pos.x, _current_pos.y);
            return i;
            }
        }
    return 0;
    }

int test_tirette(void){ //simulation
    return 1;
    }

sPt_t trjS[4]={ //trajectory of the secondary robot
    {10. , 190.},
    {40. , 160.},
    {135., 160.},
    {135., 190.}
    };

int testPtInPt(sPt_t *p1, sPt_t *p2, int r){
    return sqrt((p2->x - p1->x) * (p2->x - p1->x) + (p2->y - p1->y) * (p2->y - p1->y)) <= r ;
    }

void simuSecondary(void){ //TODO if a other robot on trajectory
    static int state = 0;
    static unsigned int lastTime = 0;
    static sPt_t pos ;

    unsigned int time = millis();
    sNum_t theta;

    if( lastTime == 0 ){
        lastTime = millis();
        pos = trjS[0];
        }

    if( state < (sizeof(trjS)/sizeof(*trjS) - 1)){
        theta = atan2( (trjS[state+1].y - trjS[state].y), (trjS[state+1].x - trjS[state].x) );
        pos.x += (SPEED_SECONDARY * (time - lastTime) * cos(theta))/1000;
        pos.y += (SPEED_SECONDARY * (time - lastTime) * sin(theta))/1000;

        if( testPtInPt(&pos, &trjS[state+1], 1) ){
            state++ ;
            }

        obs[1].r = 10;
        obs[1].c = pos;
        obs_updated[1]++;

        lastTime = millis();
        }
	}

void posPrimary(void){
    int i;
    if(get_position(&_current_pos)){
        if(((i=test_in_obs())!=0) ){
            project_point(_current_pos.x, _current_pos.y, obs[i].r, obs[i].c.x,obs[i].c.y, &_current_pos);
            if(sqrt(pow(_current_pos.x-obs[0].c.x,2)+pow(_current_pos.y-obs[0].c.y,2)<2)){
                memcpy(&obs[0].c,&_current_pos, sizeof(obs[0].c));
                obs_updated[0]++;
                }
            else{
                memcpy(&_current_pos,&obs[0].c, sizeof(obs[0].c));
                obs_updated[0]++;
                }
            }
        updateNoHaftTurn(theta_robot*180/M_PI, &obs[0].c);
        obs_updated[N-5]++;
        obs_updated[N-6]++;
        obs_updated[N-7]++;
        }
    }




