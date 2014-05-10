/*
 * obj_fire.c
 *
 *  Created on: 17 avr. 2014
 *      Author: seb
 */

#include "obj_fire.h"


eServoPos_t armLeft = CLOSE, armRight = CLOSE;

sTrajEl_t tabSeg[2]={ //Segment to push a vertical fire
	{{0.  ,  0.},{-20. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 0.},
	{{-20 ,  0.},{-20. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 1.}
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
			if(color==1)i=0;
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
			printf("Error in switch createEPfire\n");
			getchar();
			break;
		}
	}


//standard function

void obj_fire(iABObs_t  obj){
	static int state=0, theta=0; //for separate the init, loop and end
    int i;
    sTrajEl_t tabTemp[4];
    //printf("obj fire n=%d\n", obj);
     //getchar();
    switch(state){
	    case 0:
	        printf("Debut objectif feux\n\n");
	        for(i=0; i<4 ; i++){  //disable entry point
				obs[N-i-2].active=0;
				obs_updated[N-i-2]++;
	        	}

	        if(((Obj_feu*)listObj[obj].typeStruct)->pos==3){
				theta=listObj[obj].entryPoint[0].angleEP;
		        memcpy(&tabTemp[0],&tabSeg[0], sizeof(tabSeg[0])*2);
		        pt_select=listObj[obj].entryPoint[0].c;

		        obs[listObj[obj].numObs[0]].active=0;
		        obs_updated[listObj[obj].numObs[0]]++;

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
            listObj[obj].active=0;
		   break;
	    default :
	    	printf("Error in obj_tree : state != (0 || 1 || 2\n");
			break;
    	}
	}



/****Function for automatic fire****/


/*
int checkInter(sSeg_t *seg, sPath_t *path){ //recherche si il y a une intersection entre un segment et un chemin
    int i;
    sLin_t l1, l2;
    sPt_t pt1, pt2;
printf("len path : %d\n",path->path_len);
    for( i = 0 ; i < (path->path_len - 1) ; i++){
        convPts2Line(&seg->p1, &seg->p2, 0, &l1);
        convPts2Line(&path->path[i].p1, &path->path[i].p2, 0,  &l2);
        interD2D(&l1, &l2, &pt1);
        if(PtInSeg(seg , &pt1) == 1){
            return 1;
            }

        interC2D(&l1, &path->path[i].obs.c, path->path[i].obs.r, &pt1, &pt2);
        printf("Inter cercle droite : p1.x=%f, p1.y=%f, p2.x=%f, p2.y=%f\n",pt1.x, pt1.y,pt2.x, pt2.y);
        printf("In Seg ret=%d\n",PtInSeg(seg , &pt1));
        printf("In Seg ret=%d\n",PtInSeg(seg , &pt2));
        if((PtInSeg(seg , &pt1) == 1) && (PtInArc(&path->path[i].p2, &path->path[i+1].p1, &path->path[i].obs.c, &pt1) == 1) ){
            return 1;
            }
        if((PtInSeg(seg , &pt2) == 1) && (PtInArc(&path->path[i].p2, &path->path[i+1].p1, &path->path[i].obs.c, &pt2)== 1)){
            return 1;
            }
        }

    return 0;
    }
*/


void cmdServo(eServoLoc_t loc, eServoPos_t pos){
    switch(loc){ //TODO
        case ARM_LEFT :
            armLeft = pos;
            switch(pos){
                case CLOSE :
                    break;
                case HALF_OPEN :
                    break;
                case OPEN :
                    break;
                }
            break;
        case ARM_RIGHT:
            armRight = pos;
            switch(pos){
                case CLOSE :
                    break;
                case HALF_OPEN :
                    break;
                case OPEN :
                    break;
                }
            break;
        }
    }

int testPtInSseg(const sSeg_t *s, const sPt_t *p){
    sNum_t d1, d2;

    distPt2Pt(&s->p1, &s->p2, &d1);

    distPt2Pt(p, &s->p1, &d2);
    if(d1 < d2){
        return 0;
        }

    distPt2Pt(p, &s->p2, &d2);
    if(d1 < d2){
        return 0;
        }

    return 1;
    }


int checkSeg(const sSeg_t *s2, sPt_t *prev_pos, int j){
    sSeg_t s1;
    sLin_t l1, l2;
    sVec_t v1, v2;
    int ret;
    sNum_t d;
    sPt_t pt;

    //determined the intersection point
    s1.p1 = *prev_pos;
    s1.p2 = obs[0].c;
    convPts2Line(&s1.p1, &s1.p2, 0, &l1);
    convPts2Line(&s2->p1, &s2->p2, 0, &l2);

    interLine2Line(&l1, &l2, &ret, &pt);
    if(!ret){
        return 0;
        }

    //check if the point it belongs to the segments
    if(!testPtInSseg(s2, &pt)){
        return 0;
        }
    if(!testPtInSseg(&s1, &pt)){
        return 0;
        }

    //check the direction of the robot
    convPts2Vec(&s2->p1, &s2->p2, &v2);
    if(j == 0) rotVec(-M_PI_2, &v2);
    else rotVec(M_PI_2, &v2);

    convPts2Vec(&s1.p1, &s1.p2, &v1);
    dotVecs(&v1, &v2, &d);
    if(d <= 0){
        return 0;
        }

    return 1;
    }

int objBonusFire(void){
    int i, j, k, ac;
    static int first = 0;
    sPt_t p[4]; //A, B, C, D
    sSeg_t s2, seg;
    static sPt_t prev_pos ={0., 0.};
    sLin_t l;
    static int ser_active_i = -1;
    static int ser_active_j = -1;
    static int time_prev = 0;


    if(first == 0){ //initialization
        prev_pos = obs[0].c;
        cmdServo(ARM_RIGHT, CLOSE);
        cmdServo(ARM_LEFT, CLOSE);
        first = 1;
        return 0;
        }

    if( (((int)millis() - time_prev) > 3000) && (ser_active_i != -1) ){ //security
        if( ser_active_j == 0){
            cmdServo(ARM_RIGHT, CLOSE);
            }
        else{
            cmdServo(ARM_LEFT, CLOSE);
            }
        ser_active_i = -1;
        ser_active_j = -1;
        }

    if((obs[0].c.x == prev_pos.x) && (obs[0].c.y == prev_pos.y) ){ //no new position
        return 0;
        }

    if(color){
        ac=0;
        }
    else{
        ac=1;
        }

    for(i = 0 ; i < NB_OBJ ; i++){
        if((listObj[i].type == E_FEU) && (listObj[i].active == 1) && (((Obj_feu*)listObj[i].typeStruct)->pos ==3) ){
            for(j = 0 ; j< 2 ; j++){
                //1rt possibility and Right zone
                p[0].x = obs[listObj[i].numObs[0]].c.x + AZ*cos(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI*ac);
                p[0].y = obs[listObj[i].numObs[0]].c.y + AZ*sin(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI*ac);
                p[1].x = obs[listObj[i].numObs[0]].c.x + BZ*cos(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI*ac);
                p[1].y = obs[listObj[i].numObs[0]].c.y + BZ*sin(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI*ac);
                p[2].x = obs[listObj[i].numObs[0]].c.x + sqrt(BZ*BZ + BC*BC)*cos(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + atan2(BC, BZ) + M_PI*ac);
                p[2].y = obs[listObj[i].numObs[0]].c.y + sqrt(BZ*BZ + BC*BC)*sin(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + atan2(BC, BZ) + M_PI*ac);
                p[3].x = obs[listObj[i].numObs[0]].c.x + sqrt(BZ*BZ + BD*BD)*cos(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + atan2(BD, BZ) + M_PI*ac);
                p[3].y = obs[listObj[i].numObs[0]].c.y + sqrt(BZ*BZ + BD*BD)*sin(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + atan2(BD, BZ) + M_PI*ac);

                if(j == 0){ //Left zone
                    convPts2Line(&p[0], &p[1], 0, &l);
                    for( k = 2 ; k < 5 ; k++){
                        symPtprLine(&p[k], &l, &p[k]);
                        }
                    }

                for(k = 0 ; k < 2 ; k++){
                    if(k == 0) convPts2Seg(&p[0], &p[2], &s2);
                    if(k == 1) convPts2Seg(&p[2], &p[3], &s2);

                    if(!checkSeg(&s2, &prev_pos, j)){
                        continue;
                        }

                    printf("prev_pos.x = %f, prev_pos.y=%f\n", prev_pos.x, prev_pos.y);
                    printf("pos.x = %f, pos.y=%f\n", obs[0].c.x, obs[0].c.y);
                    printf("i=%d j=%d\n",i,j);

                    if( j == 0){
                        cmdServo(ARM_RIGHT, HALF_OPEN);
                        }
                    else{
                        cmdServo(ARM_LEFT, HALF_OPEN);
                        }
                    ser_active_i = i;
                    ser_active_j = j;
                    time_prev = millis();
                    }

                //A first line fire already detect
                if( (ser_active_i == i) && (ser_active_j == j) ){
                    seg.p1.x = obs[listObj[i].numObs[0]].c.x + EZ*cos(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI_2 + M_PI*ac);
                    seg.p1.y = obs[listObj[i].numObs[0]].c.y + EZ*sin(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI_2 + M_PI*ac);
                    seg.p2.x = obs[listObj[i].numObs[0]].c.x + BD*cos(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI_2 + M_PI*ac);
                    seg.p2.y = obs[listObj[i].numObs[0]].c.y + BD*sin(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI_2 + M_PI*ac);

                    if(j == 0){
                        symPtprLine(&seg.p1, &l, &seg.p1);
                        symPtprLine(&seg.p2, &l, &seg.p2);
                        }

                    if(checkSeg(&seg, &prev_pos, j)){
                        if( j == 0){
                            cmdServo(ARM_RIGHT, CLOSE);
                            }
                        else{
                            cmdServo(ARM_LEFT, CLOSE);
                            }
                        listObj[i].active = 0;
                        if(color == 1){
                            ((Obj_feu*)listObj[i].typeStruct)->pos = 2;
                            }
                        else{
                            ((Obj_feu*)listObj[i].typeStruct)->pos = 1;
                            }
                        ser_active_i = -1;
                        ser_active_j = -1;
                        }
                    }

                //2nd possibility
                seg.p1.x = obs[listObj[i].numObs[0]].c.x + EZ*cos(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI_2 + M_PI*ac);
                seg.p1.y = obs[listObj[i].numObs[0]].c.y + EZ*sin(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI_2 + M_PI*ac);
                seg.p2.x = obs[listObj[i].numObs[0]].c.x + BD*cos(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI_2 + M_PI*ac);
                seg.p2.y = obs[listObj[i].numObs[0]].c.y + BD*sin(((Obj_feu*)listObj[i].typeStruct)->angle*M_PI/180 + M_PI_2 + M_PI*ac);

                if(j == 0){
                    symPtprLine(&seg.p1, &l, &seg.p1);
                    symPtprLine(&seg.p2, &l, &seg.p2);
                    }

                if(j == 0) k = 1;
                else k = 0;

                if(!checkSeg(&seg, &prev_pos, k)){
                    continue;
                    }

                if( j == 0){
                    cmdServo(ARM_LEFT, OPEN);
                    ser_active_j = 1;
                    }
                else{
                    cmdServo(ARM_RIGHT, OPEN);
                    ser_active_j = 0;
                    }
                ser_active_i = i;
                time_prev = millis() - 2000;
                listObj[i].active = 0;
                if(color == 1){
                    ((Obj_feu*)listObj[i].typeStruct)->pos = 2;
                    }
                else{
                    ((Obj_feu*)listObj[i].typeStruct)->pos = 1;
                    }
                }
            }
        }

    prev_pos = obs[0].c;
    return 1;
    }
