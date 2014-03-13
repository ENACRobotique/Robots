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

#include"type_ia.h"

sPt_t _current_pos;
long _start_time;
long last_time=0;
long last_time2=0;
sPath_t path= {.dist = 0.,  .path = NULL };

Obj_arbre arbre[4];
Obj_bac bac;
Obj_feu feu[4];


sObj_t listObj[NB_OBJ];
                /*=
{
	{E_ARBRE, 0, 1,0,0,1,4,0,&arbre[0]},
	{E_ARBRE, 1, 1,0,0,1,4,0,&arbre[1]},
	{E_ARBRE, 2, 1,0,0,1,4,0,&arbre[2]},
	{E_ARBRE, 3, 1,0,0,1,4,0,&arbre[3]},
	{E_BAC,   0, 1,0,0,1,4,0,&bac}
	};*/




sObs_t listEP[]={
	//Arbre
	{{10. , 90.}, 0 ,1,1},
	{{10. , 50.}, 0 ,1,1},
	{{20. , 90.}, 0 ,1,1},
	{{20. , 50.}, 0 ,1,1},

	{{50. , 10.}, 0 ,1,1},
	{{90. , 10.}, 0 ,1,1},
	{{50. , 20.}, 0 ,1,1},
	{{90. , 20.}, 0 ,1,1},

	{{210. ,10.}, 0 ,1,1},
	{{250. ,10.}, 0 ,1,1},
	{{210. ,20.}, 0 ,1,1},
	{{250. ,20.}, 0 ,1,1},

	{{290. ,50.}, 0 ,1,1},
	{{290. ,90.}, 0 ,1,1},
	{{280. ,50.}, 0 ,1,1},
	{{280. ,90.}, 0 ,1,1},

	//Bac
	{{225.,170.},0,1,1}, //rouge

	{{75. ,170.},0,1,1}, //jaune

	//Feux (centre)
	{{40. ,  90. }, 0, 1, 1},
	{{90. , 40.  }, 0, 1, 1},
	{{90. , 140. }, 0, 1, 1},
	{{210., 40.  }, 0, 1, 1},
	{{210., 140. }, 0, 1, 1},
	{{260.,  90. }, 0, 1, 1},

	{{1.  , 120. }, 0, 1, 1},
	{{130. , 1.  }, 0, 1, 1},
	{{170. , 1.  }, 0, 1, 1},
	{{299., 120. }, 0, 1, 1},

	{{90. , 90.  }, 0, 1, 1},
	{{90. , 90.  }, 0, 1, 1},
	{{90. , 90.  }, 0, 1, 1},

	{{210. ,90. }, 0, 1, 1},
	{{210. ,90. }, 0, 1, 1},
	{{210. ,90. }, 0, 1, 1},

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
	int i;
	printf("ListObj :\n");
	for(i=0 ; i<NB_OBJ ; i++)
		{
		printf("type=%i, numObj=%i, nbObs=%i,{listIABObs[0]=%i,listIABObs[1]=%i,listIABObs[2]=%i} dist=%f, active=%i, nbEP=%i\n",listObj[i].type,listObj[i].numObj,listObj[i].nbObs,listObj[i].listIABObs[0],listObj[i].listIABObs[1],listObj[i].listIABObs[2],listObj[i].dist,listObj[i].active,listObj[i].nbEP);
		}
	printf("\n");
	}

void init_ele(void)
	{
	printf("Debut de l'initialisation des elements du jeu\n");
	int i, j, numPA=0;

	for(i=0; i<N;i++ ) obs[i].active =1; //activation de tous les obstacles

	//Initialisation des arbres
		for(i=0 ; i<4 ; i++)
			{
			listObj[i].type=E_ARBRE;
			listObj[i].numObj=i;
			listObj[i].nbObs=1;
			listObj[i].listIABObs[0]=i+1;
			listObj[i].dist=0.;
			listObj[i].active=1;
			listObj[i].nbEP=4;
			for(j=0 ; j<4 ; j++, numPA++) listObj[i].entryPoint[j]=listEP[numPA];
				listObj[i].entryPoint[0].active=1;
				listObj[i].entryPoint[1].active=1;
				listObj[i].entryPoint[2].active=0;
				listObj[i].entryPoint[3].active=0;
			listObj[i].typeStruct = &arbre[listObj[i].numObj];

			((Obj_arbre*)listObj[i].typeStruct)->nb_point=10;  					   //1 fruit pouri par arbre
			for(j=0 ; j<6 ; j++) ((Obj_arbre*)listObj[i].typeStruct)->eFruit[j]=0; //par défaut tous les fruit sont bon
			}
	//Initialisation du bac
		listObj[i].type=E_BAC;
		listObj[i].numObj=0;
		listObj[i].nbObs=1;
		if(COLOR==0) listObj[i].listIABObs[0]=10;
		else listObj[i].listIABObs[0]=5;
		listObj[i].dist=0.;
		listObj[i].active=1;
		listObj[i].nbEP=1;
		if(COLOR==1) numPA++;
		listObj[i].entryPoint[0]=listEP[numPA];
			listObj[i].entryPoint[0].active=1;
		listObj[i].typeStruct = &bac;

		((Obj_bac*)listObj[i].typeStruct)->nb_point=0;
		i++;
		numPA++;
		if(COLOR==0) numPA++;

	//Initialisation des feux vericaux sur la table +  coté
		for(i=5 ; i<11 ; i++)
			{
			listObj[i].type=E_FEU;
			listObj[i].numObj=i-5;
			listObj[i].nbObs=2;

			for(j=0 ; j<listObj[i].nbObs ; j++)
				{
				listObj[i].listIABObs[j]=26+j+2*(i-5);
				}
			listObj[i].dist=0.;
			listObj[i].active=1;
			listObj[i].nbEP=1;
			listObj[i].entryPoint[0]=listEP[numPA];
				listObj[i].entryPoint[0].active=1;
			numPA++;
			listObj[i].typeStruct = &feu[i];

			((Obj_feu*)listObj[i].typeStruct)->pos=0;
			((Obj_feu*)listObj[i].typeStruct)->nb_point=2;
			}

		//Initialisation des feux vericaux sur le  coté
			for(i=11 ; i<15 ; i++)
				{
				listObj[i].type=E_FEU;
				listObj[i].numObj=i-5;
				listObj[i].nbObs=3;

				if(i==11)
					{
					listObj[i].listIABObs[0]=20;
					listObj[i].listIABObs[1]=21;
					listObj[i].listIABObs[2]=38;
					}
				if(i==12)
					{
					listObj[i].listIABObs[0]=22;
					listObj[i].nbObs=2;
					listObj[i].listIABObs[1]=39;
					}
				if(i==13)
					{
					listObj[i].listIABObs[0]=23;
					listObj[i].nbObs=2;
					listObj[i].listIABObs[1]=40;
					}
				if(i==14)
					{
					listObj[i].listIABObs[0]=24;
					listObj[i].listIABObs[1]=25;
					listObj[i].listIABObs[2]=41;
					}

				listObj[i].dist=0.;
				listObj[i].active=1;
				listObj[i].nbEP=1;
				listObj[i].entryPoint[0]=listEP[numPA];
					listObj[i].entryPoint[0].active=1;
				numPA++;
				listObj[i].typeStruct = &feu[i];

				((Obj_feu*)listObj[i].typeStruct)->pos=0;
				((Obj_feu*)listObj[i].typeStruct)->nb_point=2;
				}

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

    		outMsg.header.destAddr = ADDRD_MAIN_PROP_SIMU;
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

    		ret = bn_sendAck(&outMsg);
    		if(ret < 0){
    			printf("bn_send() failed #%i\n", -ret);
    		}

    		outMsg.header.destAddr = ADDRD_MONITORING;
    		ret = bn_sendAck(&outMsg);
    		if(ret < 0){
    			printf("bn_send() failed #%i\n", -ret);
    			sleep(10);
    		}
    		printf("\n");
    		usleep(1000);
    	}
	}

int get_position( sPt_t *pos)
	{
//	pos->c.x=obs_PA[2*goal].c.x;
//	pos->c.y=obs_PA[2*goal].c.y;
	sMsg inMsg;
	int ret;
	ret = bn_receive(&inMsg);
	if(ret < 0)
		{
		printf("bn_receive() error #%i\n", -ret);
		}
	else if(ret > 0)
		{
		switch(inMsg.header.type)
			{
			case E_POS:
				pos->x=inMsg.payload.pos.x;
				pos->y=inMsg.payload.pos.y;
				//printf("received position (%.1fcm,%.1fcm,%.2f°)\n", inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta*180./M_PI);

				memcpy(&outMsg, &inMsg, sizeof(inMsg.header)+inMsg.header.size);
				outMsg.header.destAddr = ADDRD_MONITORING;
				ret = bn_send(&outMsg);
				if(ret < 0){
					printf("bn_send() error #%i\n", -ret);
				}

				return 1;
				break;
			default:
				//printf("received unknown messsage, type %s (%i)\n", eType2str(inMsg.header.type), inMsg.header.type);
				break;
			}
    	}
	return 0;
	}

float sign(float x)
	{
	if(x==0) return 0;
	return x/fabs(x);
	}

void project_point(float xp, float yp, float rc, float xc, float yc, sPt_t *point) //Ajout de 0.1
	{
	float alpha;
	if((xp-xc)==0)
		{
		point->x = xc;
		point->y = yc+rc;
		}
	else
		{
		if((xp-xc)>0)
			{
			alpha=atan((yp-yc)/(xp-xc));
//			printf("return sign =%f et alpha=%f\n",sign(xp-xc), alpha*180/M_PI);
			point->x = xc + rc*cos(alpha);
			point->y = yc + rc*sin(alpha);
			if((yp-yc)>0)
				{
				point->x = point->x + 0.1;
				point->y = point->y + 0.1;
				}
			if((yp-yc)<0)
				{
				point->x = point->x + 0.1;
				point->y = point->y - 0.1;
				}
			}
		if((xp-xc)<0)
			{
			alpha=atan((yp-yc)/(xp-xc));
//			printf("return sign =%f et alpha=%f\n",sign(xp-xc), alpha*180/M_PI);
			point->x = xc + rc*cos(alpha+M_PI);
			point->y = yc + rc*sin(alpha+M_PI);
			if((yp-yc)>0)
				{
				point->x = point->x - 0.1;
				point->y = point->y + 0.1;
				}
			if((yp-yc)<0)
				{
				point->x = point->x - 0.1;
				point->y = point->y - 0.1;
				}
			}
		}
  	}

int init_mes(void)
	{
	int ret;
	ret = bn_init();
		if(ret < 0)
			{
			printf("bn_init() failed #%i\n", -ret);
			return(1);
			}

	    outMsg.header.destAddr = ADDRD_MAIN_PROP_SIMU;
	    outMsg.header.type = E_POS;
	    outMsg.header.size = sizeof(outMsg.payload.pos);
	    outMsg.payload.pos.id = 0;
	    outMsg.payload.pos.theta = 0;
	    outMsg.payload.pos.u_a = 0;
	    outMsg.payload.pos.u_a_theta = 0;
	    outMsg.payload.pos.u_b = 0;
	    outMsg.payload.pos.x = obs[0].c.x;
	    outMsg.payload.pos.y = obs[0].c.y;
		printf("Sending initial position to robot%i (%fcm,%fcm,%f°).\n", outMsg.payload.pos.id, outMsg.payload.pos.x, outMsg.payload.pos.y, outMsg.payload.pos.theta*180./M_PI);
	    ret = bn_sendAck(&outMsg);
	    if(ret <= 0)
	    	{
	        printf("bn_sendAck() error #%i\n", -ret);
	        return(1);
	    	}
	    return(0);
	}
