/*
 * obj_fct.c
 *
 *  Created on: 29 mars 2014
 *      Author: seb
 */

#include "obj_fct.h"

#include "obj_com.h"

void printServoPos(eServoPos_t *pos){
    switch(*pos){
        case CLOSE:
            printf("CLOSE");
            break;
        case HALF_OPEN:
            printf("HALF_OPEN");
            break;
        case OPEN:
            printf("OPEN");
            break;
        default:
            printf("Error in switch printServoPos\n");
        }
    }

void printListObj(void){
    int i,j;
    printf("ListObj :\n");
    for(i=0 ; i<NB_OBJ ; i++){
        printf("type=%d, ",listObj[i].etype);
        printf("numObj=%d, ",listObj[i].numObj);
        printf("nbObs=%d, ",listObj[i].nbObs);
        printf("numObs={");
        for(j=0 ; j<listObj[i].nbObs ; j++) printf("%d, ",listObj[i].numObs[j]);
        printf("}, ");
        printf("dist=%f, ",listObj[i].dist);
        printf("active=%d, ",listObj[i].active);
        printf("done=%f, ",listObj[i].done);
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

void printPath(sPath_t *path){
    int i;

    for(i = 0 ; i < path->path_len ; i++){
        printf("{{%f, %f},{%f, %f},{{%f, %f}, %f, %d , %d, %d}, %f , %f, %d}\n", path->path[i].p1.x, path->path[i].p1.y, path->path[i].p2.x, path->path[i].p2.y, path->path[i].obs.c.x,path->path[i].obs.c.y, path->path->obs.r, path->path->obs.moved, path->path[i].obs.active, path->path[i].obs.state, path->path[i].arc_len, path->path[i].seg_len, path->path[i].sid );
        }
    }

void printObsActive(void){
	int i;
	printf("Liste des obs[i].active :\n");
	for(i=0 ; i<N ; i++)
		printf("obs[%d].active=%d\n",i,obs[i].active);
	printf("\n");
	}

int initTraj(void){
    sVec_t v;
    sPath_t path;
    sPt_t p;
    sNum_t theta;
    static int state = 0;
    static sWaitPos waiting_pos = {0};
    static unsigned long setpos_start = 0;
  /*  sTrajEl_t trajRed[2]={
        {{0. , 0.},{12.9 + 5. , 0.},{{12.9 + 5. ,  0.}, 0. , 0., 1.}, 0. , 0., 0},
        {{12.9 + 18., 0.},{12.9 + 18., 0.},{{12.9 + 18. ,  0.}, 0. , 0., 1.}, 0. , 0., 1},
        };*/
    sTrajEl_t trajRed[2]={
        {{0.            , 0.},{12.9 + 3. - 2., 0.},{{12.9 + 3. - 2. ,  0.}, 5. , 0., 1.}, 0. , 0., 0},
        {{12.9 + 8. - 2., 0.},{12.9 + 8. - 2., 0.},{{12.9 + 8. - 2. ,  0.}, 0. , 0., 1.}, 0. , 0., 1},
        };
    sTrajEl_t trajYellow[2]={
        {{0.                   , 0.},{300. - 12.9 - 3. + 2., 0.},{{300. - 12.9 - 3. + 2. ,  0.}, -5. , 0., 1.}, 0. , 0., 0},
        {{300. - 12.9 - 8. + 2., 0.},{300. - 12.9 - 8. + 2., 0.},{{300. - 12.9 - 8. + 2. ,  0.}, 0. , 0., 1.}, 0. , 0., 1},
        };
    sTrajEl_t traj[2];

    switch(state){
        case 0 :
        if(color == 1){ //yellow
            v.x = 10;
            v.y = 0;
            }
        else{
            v.x = -10;
            v.y = 0;
            }
        setpos_start = 0;
        newSpeed(- LOW_SPEED);
        sendSeg(NULL, &v);
        pt_select.x = obs[0].c.x + v.x;
        pt_select.y = obs[0].c.y + v.y;
        state = 1;
        break;

        case 1:
            if ((fabs(obs[0].c.x - pt_select.x) < 1. && fabs(obs[0].c.y - pt_select.y) < 1.)){
                if(!setpos_start){
                    setpos_start = millis();
                    break;
                }
                else{
                    if(millis() - setpos_start > 200){
                        if(color == 1){ // yellow
                            p.x = 300. - 12.9;
                            theta = M_PI;
                        }
                        else{ // red
                            p.x = 12.9;
                            theta = 0.;
                        }
                        p.y = obs[0].c.y;

                        waiting_pos.next = 2;
                        waiting_pos.pos = p;
                        waiting_pos.theta = theta;

                        setPos(&p, theta);
                        state = 10;
                    }
                }
            }
            break;

        case 10: // waiting pos
        {
            sNum_t dist;

            distPt2Pt(&waiting_pos.pos, &obs[0].c, &dist);

            if(dist <= 1.  /* XXX test theta aswell */){
                state = waiting_pos.next;
            }
            break;
        }

        case 2 :
            if( color == 1){ //yellow
                memcpy(traj, trajYellow, sizeof(trajYellow));
                }
            else{
                memcpy(traj, trajRed, sizeof(trajRed));
                }
//printf("cur .x = %f .y=%f\n",obs[0].c.x, obs[0].c.y);

            traj[0].p1 = obs[0].c;
            traj[0].p2.y = obs[0].c.y;
            traj[0].obs.c.y = obs[0].c.y - 5.;
            traj[1].p1.y = obs[0].c.y - 5.;
            traj[1].p2.y = obs[0].c.y - 10.;
            traj[1].obs.c.y = obs[0].c.y - 10.;


            newSpeed(LOW_SPEED);
            path_len(traj, 2);
            traj[1].seg_len = 3.;
            path.path_len = 2;
            path.path = traj;

            send_robot(path);
            pt_select = traj[1].p2;

            state = 3;
            break;

        case 3 :
            if ((fabs(obs[0].c.x - pt_select.x) < 1. && fabs(obs[0].c.y - pt_select.y) < 1.)){
                state = 4;
                }
            break;

        case 4 :
            v.x = 0;
            v.y = 30;
            newSpeed(- LOW_SPEED);
            sendSeg(NULL, &v);
            setpos_start = 0;
            pt_select.x = obs[0].c.x + v.x;
            pt_select.y = obs[0].c.y + v.y;

            state = 5;
            break;

        case 5 :
            if ((fabs(obs[0].c.x - pt_select.x) < 1. && fabs(obs[0].c.y - pt_select.y) < 1.)){
                if(!setpos_start){
                    setpos_start = millis();
                    break;
                }
                else{
                    if(millis() - setpos_start > 200){
                        p.x = obs[0].c.x;
                        p.y = 200 - 12.9;
                        theta = -M_PI_2;
                        setPos(&p, theta);

                        waiting_pos.next = 6;
                        waiting_pos.pos = p;
                        waiting_pos.theta = theta;

                        state = 50;
                    }
                }
            }
            break;

        case 50: // waiting pos
        {
            sNum_t dist;

            distPt2Pt(&waiting_pos.pos, &obs[0].c, &dist);

            if(dist <= 1. /* XXX test theta aswell */){
                state = waiting_pos.next;
            }
            break;
        }

        case 6:
            v.x = 0;
            v.y = -2;
            newSpeed(LOW_SPEED);
            sendSeg(NULL, &v);
            pt_select.x = obs[0].c.x + v.x;
            pt_select.y = obs[0].c.y + v.y;

            state = 7;
            break;

        case 7 :
            if ((fabs(obs[0].c.x - pt_select.x) < 1. && fabs(obs[0].c.y - pt_select.y) < 1.)){
                state = 8;
                pt_select.x = 0;
                pt_select.y = 0;
                newSpeed(NOMINAL_SPEED);
                _current_pos = obs[0].c;
                }
            break;
        default :
            printf("Error in swich\n");
            getchar();
            break;
        }

    if(state == 8){
        state = 0;
        return 1;
        }
    else return 0;

    }


void init_ele(void){
    printf("Debut de l'initialisation des elements du jeu\n");
    int i, j;

    //Initialisation des arbres
    for(i=0 ; i<4 ; i++){
    	listObj[i].utype.tree.nb_point=10;
    	listObj[i].entryPoint[0].radiusEP = RADIUS_ENTRY_POINT_TREE;
        listObj[i].entryPoint[1].radiusEP = RADIUS_ENTRY_POINT_TREE;
    	for(j=0 ; j<6 ; j++) listObj[i].utype.tree.eFruit[j]=0;   //default all fruits are good
    	}
    updateEntryPointTree();

    //Initialisation des bacs
    for(i=4 ; i<6 ; i++){
    	//listObj[i].typeStruct = &bac;
        listObj[i].utype.basket.nb_point=0;
    	}
    if(color==1)listObj[4].active=0;
    else listObj[5].active=0;


    //Initialisation des feux
    for(i=6 ; i<16 ; i++){
    	//listObj[i].typeStruct = &feu[listObj[i].numObj];
        listObj[i].utype.fire.nb_point=2;
        listObj[i].nbEP=3;
        createEPfire2(i);
        }

    if(color == RED){
        listObj[13].active = 0;
        listObj[15].active = 0;
        }
    else{
        listObj[12].active = 0;
        listObj[14].active = 0;
        }

	#if DEBUG
		printListObj();
	#endif
    printf("Fin de l'initialisation des elements du jeu\n");
    //getchar();
    }


int get_position( sPt_t *pos){
	*pos = obs[0].c;
    obs_updated[0]++;
	return 1;
    }

int test_in_obs(sPt_t *p){ //retourne le numéros de l'obstable si la position est a l'interieur de celui ci
    //FIXME si le robot dans plusieurs obstable
    int i;
    sNum_t dist;
    for(i=1; i<N-1;i++){
        if(obs[i].active==0)continue;
        distPt2Pt(&obs[i].c, p, &dist);
        if(  dist < obs[i].r){
        	//printf("Le robot est dans l'obstacle n=%i, robs=%f, xobs=%f, yobs=%f, currentpos x=%f, y=%f\n",i,obs[i].r,obs[i].c.x,obs[i].c.y, _current_pos.x, _current_pos.y);
            return i;
            }
        }
    return 0;
    }

int test_tirette(void){
    #if SIMU
        return 1;
    #endif
    return starting_cord;
}

void startColor(void){

#if SIMU
    color = COLOR_SIMU;
#else
    static int state = 0;
    sMsg msgOut;

    switch(state){
        case 0 :
            if(mode_switch == 0){ //Red color
                state = 1;
                color = 0;

                msgOut.header.destAddr = ADDRI_MAIN_IO;
                msgOut.header.type = E_IHM_STATUS;
                msgOut.header.size = 2 + 1*sizeof(*msgOut.payload.ihmStatus.states);

                msgOut.payload.ihmStatus.nb_states = 1;
                msgOut.payload.ihmStatus.states[0].id = IHM_LED;
                msgOut.payload.ihmStatus.states[0].state = 1;

                bn_sendRetry(&msgOut, MAX_RETRIES);
                }
            break;
        case 1 :

            if(mode_switch == 1){
                state = 2;
                }
            break;
        case 2 :
            if(mode_switch == 0){ //Yellow color
                state = 3;
                color = 1;

                msgOut.header.destAddr = ADDRI_MAIN_IO;
                msgOut.header.type = E_IHM_STATUS;
                msgOut.header.size = 2 + 1*sizeof(*msgOut.payload.ihmStatus.states);

                msgOut.payload.ihmStatus.nb_states = 1;
                msgOut.payload.ihmStatus.states[0].id = IHM_LED;
                msgOut.payload.ihmStatus.states[0].state = 2;

                bn_sendRetry(&msgOut, MAX_RETRIES);
                }
            break;
        case 3 :
            if(mode_switch == 1){
                state = 0;
                }
            break;
        }
#endif
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

    if( !lastTime ){
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

        obs[1].c = pos;
        obs_updated[1]++;

        lastTime = millis();
        }
	}

void posPrimary(void){
    int i;
    sPt_t pt;
    sVec_t v;

    if(get_position(&_current_pos)){
        if(((i=test_in_obs(&_current_pos))!=0) ){
            if( obs[i].moved == 1){
                pt = obs[0].c;
                projPtOnCircle(&obs[i].c, obs[i].r, &pt);
                convPts2Vec(&pt, &obs[0].c, &v);

                obs[i].c.x += v.x;
                obs[i].c.y += v.y;

                obs_updated[i]++;
                }
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

void updateRatioObj(int numObj, int robot){ //robot = 1 to 3
    static int mat[3][NB_OBJ] = {{0, 0}};  //3 is the number of other robot

    if( !mat[robot-1][numObj] ){
        listObj[numObj].done += (1 - listObj[numObj].done)/2;
        mat[robot-1][numObj] = 1;
        }
    }

void checkRobot2Obj(void){
    int i, j, k;
    sNum_t dist;
    static int tab[NB_OBJ][2] ={{0, 0}};

    for(i = 0 ; i < NB_OBJ ; i++){
        if(tab[i][0]){
            tab[i][0] = 0;
            listObj[i].active = tab[i][1];
            }
        for(j = 1 ; j < 4 ; j++){
            for(k = 0 ; k < listObj[i].nbObs ; k++){
                distPt2Pt(&obs[listObj[i].numObs[k]].c, &obs[j].c, &dist);
                if( (obs[listObj[i].numObs[k]].r + obs[j].r - R_ROBOT) > (dist - ERR_DIST) ){
                    updateRatioObj(i, j);
                    tab[i][0] = 1;
                    tab[i][1] = listObj[i].active;
                    listObj[i].active = 0;
                    }
                }
            }
        }
    }

int checkAdvOnRobot(void){
    int i;
    sNum_t dist;

    for(i = 1 ; i < 3 ; i++){
        distPt2Pt(&obs[i].c, &obs[0].c, &dist);
        if( (obs[i].r) > (dist - ERR_DIST) ){
            printf("On est dans un robot\n");
            return 1;
            }
        }

    return 0;
    }

int checkRobotBlock(void){
    static sPt_t pos[10] ={{0., 0.}};
    static int pt = 0;
    static unsigned int lastTime = 0;
    int i, cpt = 0;
    sNum_t dist;

    if( fabs(time_diff(millis(), lastTime)) > 200){
        pos[pt] = obs[0].c;
        pt++;
        pt = pt%10;
        for(i = 0 ; i < 10 ; i++){
            distPt2Pt(&obs[0].c, &pos[i], &dist);
            if(dist < 1.) cpt++;
            }
        if(cpt == 10){
            //printf("Warning robot block\n");
            return 1;
            }
        lastTime = millis();
        }

    return 0;
    }

void stopRobot(void){
    sPath_t path;
    sTrajEl_t traj;

    traj.p1 = obs[0].c;
    traj.p2 = obs[0].c;
    traj.arc_len = 0.;
    traj.seg_len = 0.;
    traj.sid = 0;

    path.path_len = 1;
    path.path = &traj;

    send_robot(path);
    }

//TODO Optimisation des deplacement du robot algarithme arbre recouvrant

