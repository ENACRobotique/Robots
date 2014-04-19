/*
 * obj_fire.c
 *
 *  Created on: 17 avr. 2014
 *      Author: seb
 */

#include "obj_fire.h"



sTrajEl_t tabSeg[2]={ //Segment for push a vertical fire
	{{0.  ,  0.},{-10. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 0.},
	{{-10 ,  0.},{-10. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 1.}
	};


//Entry point fire

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


//standard function

void obj_fire(iABObs_t  obj){
	static int state=0, theta=0; //for separate the init, loop and end
    int i;
    sTrajEl_t tabTemp[4];

    switch(state){
	    case 0:
	        printf("Debut objectif feux\n\n");
	        for(i=0; i<4 ; i++){  //disable entry point
				obs[N-i-2].active=0;
				obs_updated[N-i-2]++;
	        	}
	        obs[listObj[obj].numObs[0]].active=0;
	        obs_updated[listObj[obj].numObs[0]]++;

	        listObj[obj].active=0;

	        if(((Obj_feu*)listObj[obj].typeStruct)->pos==3){
				theta=listObj[obj].entryPoint[0].angleEP;
		        memcpy(&tabTemp[0],&tabSeg[0], sizeof(tabSeg[0])*2);
		        pt_select=listObj[obj].entryPoint[0].c;

				//Rotation du segment

				tabTemp[0].p2.x= -10*cos(theta*M_PI/180);
				tabTemp[0].p2.y= -10*sin(theta*M_PI/180);

				tabTemp[1].p1=tabTemp[0].p2;
				tabTemp[1].p2=tabTemp[0].p2;

				//Translation du segment
				TransElTraj(&tabTemp[0], listObj[obj].entryPoint[0].c.x, listObj[obj].entryPoint[0].c.y);
				tabTemp[1].p1=tabTemp[0].p2;
				tabTemp[1].p2=tabTemp[0].p2;

				//Update extremum
	            pt_select=(tabTemp[1]).p1;
	            tabTemp[0].p1=obs[0].c;

	            //send path
	            printf("p1.x=%f, p1.y=%f, p2.x=%f, p2.y=%f \net  p1.x=%f, p1.y=%f, p2.x=%f, p2.y=%f\n",tabTemp[0].p1.x,tabTemp[0].p1.y,tabTemp[0].p2.x,tabTemp[0].p2.y,tabTemp[1].p1.x,tabTemp[1].p1.y,tabTemp[1].p2.x,tabTemp[1].p2.y);
	            path.path=&tabTemp[0];
	            path.path_len=2;
	            send_robot(path) ;

	            obs[N-1].c=pt_select;
	            obs_updated[N-1]++;

	            obs[obj+1].r=R_ROBOT + 5;
	            obs_updated[obj+1]++;

				}

            //generique fonction
    		last_time=millis();
    		mode_obj=1;
    		state=1;
    		//printf("Stop getchar\n");
    		//getchar();
            printf("start x=%f y=%f end x=%f y=%f\n",obs[0].c.x,obs[0].c.y,obs[N-1].c.x,obs[N-1].c.y);
	        break;
	    case 1:
			if ((fabs(pt_select.x-_current_pos.x)<1 && fabs(pt_select.y-_current_pos.y)<1)){
				state=2;
				}
	    	break;
	    case 2 :
	        if(((Obj_feu*)listObj[obj].typeStruct)->pos==3){
				((Obj_feu*)listObj[obj].typeStruct)->c.x += -10*cos(theta*M_PI/180);
				((Obj_feu*)listObj[obj].typeStruct)->c.y += -10*sin(theta*M_PI/180);
				obs[listObj[obj].numObs[0]].c.x=((Obj_feu*)listObj[obj].typeStruct)->c.x ;
				obs[listObj[obj].numObs[0]].c.y=((Obj_feu*)listObj[obj].typeStruct)->c.y ;
		        obs[listObj[obj].numObs[0]].active=1;
		        //printf("Rayon=%f",obs[listObj[obj].numObs[0]].r);
		        //getchar();
				obs_updated[listObj[obj].numObs[0]]++;
				}

		    mode_obj=0;
		    state=0;
		    pt_select.x=0;
		    pt_select.y=0;
		    listObj[obj].dist=0;
		   break;
	    default :
	    	printf("Error in obj_tree : state != (0 || 1 || 2\n");
			break;
    	}
	}
