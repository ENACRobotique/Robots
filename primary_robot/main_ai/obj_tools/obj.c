#include "obj.h"

#include <math.h>

#include "obj_com.h"
#include "main.h"

#define CLAMP(m, v, M) MAX(m, MIN(v, M))


estate_t state = COLOR_SELECTION;
int temp=0; //Temporaire pour mettre en shutdown lorsque tous les objectifs sont finis

int prev_obj=-1;
sPt_t prev_pos={0., 0.};
sNum_t prev_len=0;
long last_time2=0;


void updateEndTraj(sNum_t theta, sPt_t *pt, sNum_t r){
	int i;
	for(i=1 ; i<4 ; i++){
		obs[N-i-1].c.x=pt->x+(r)*cos(theta*M_PI/180+i*M_PI_2);
		obs[N-i-1].c.y=pt->y+(r)*sin(theta*M_PI/180+i*M_PI_2);
		obs[N-i-1].active=1;
		obs[N-i-1].r=r-0.5;
		}
	}

void updateNoHaftTurn(sNum_t theta, sPt_t *pt){
	int i;
	sNum_t r;
	r=speed/3;
	if(r>15)r=15;

	for(i=1 ; i<4 ; i++){
		obs[N-i-4].c.x=pt->x+(r)*cos(theta*M_PI/180+i*M_PI_2);
		obs[N-i-4].c.y=pt->y+(r)*sin(theta*M_PI/180+i*M_PI_2);
		obs[N-i-4].active=1;
		obs[N-i-4].r=r-0.5;
		}
	if(r<0.1){
		for(i=1 ; i<4 ; i++)obs[N-i-4].active=0;
		}
	}

sNum_t val_obj(int num){ //numeros de l'objectif dans listObj[] compris entre 0 et NB_OBJ
    printf("Debut val_obj avec num=%i\n",num);

    sNum_t speed=1;
    sNum_t time;
    sNum_t dist;
    sNum_t point;
    sNum_t ratio = 0.;

    dist=listObj[num].dist;
    if(dist==0){
        printf("Attention : distance nul\n"); //Erreur ou objectif atteint
        return (-1);
        }
    time=dist/speed;
    if(time > (END_MATCH-(millis()-_start_time))){ //temps restant insuffisant
        listObj[num].active = 0;
        return 0;
        }

    switch(listObj[num].etype){
        case E_FEU :
            printf("enum=%i => type=E_FEU, dist=%f\n",listObj[num].etype,dist);
            point = listObj[num].utype.fire.nb_point;
            ratio = point/time*100;
            break;

        case E_TORCHE_MOBILE :
            break;

        case E_ARBRE :
            printf("enum=%i => type=E_ARBRE, dist=%f\n",listObj[num].etype,dist);
            point = listObj[num].utype.tree.nb_point;
            ratio = point/time*1000+ratio_arbre() ;
            printf("ratio_arbre()=%f et n=%d\n",ratio_arbre(),num);
            break;

        case E_BAC :
            printf("enum=%i => type=E_BAC, dist=%f\n",listObj[num].etype,dist);
            point = listObj[num].utype.basket.nb_point;
            ratio = point/time*1000+ratio_bac();
            break;

        case E_FOYER :
            break;

        case E_TORCHE_FIXE :
            break;

        default:
        	return(-1);
        	break;
        }
    return ratio * (1. - listObj[num].done);
    }



int8_t next_obj (void){
    printf("Debut next_obj\n");

    sNum_t tmp_val = 0.;
    sPath_t path_loc={.dist = 0.,  .path = NULL };
    sPath_t path_loc2={.dist = 0.,  .path = NULL };
    sNum_t tmp_val2;
    sNum_t dist=999.;
    sPt_t pointactuel={0,0};
    float r = 0., rt = 0.;
    sPt_t pt1, pt2, pt3;
    sPt_t pt1t, pt2t, pt3t;
    int k=0;
    int i,j, tmp_inx=-1, g; //tmp_inx : index de l'objectif qui va  etre selectionner
    int num_obs;
    int tabFeux[16] = {0};
    int m;


    obs[N-1].active=1;

    for(i = 0 ; i < NB_OBJ ; i++){
        if (listObj[i].active==0) continue; //test if  objective is still active

		#if DEBUG
				//printObsActive();
		#endif

        for(j=0 ; j<listObj[i].nbEP ; j++){ //find the optimal access point of this objective and its trajectory
            memcpy(&obs[N-1].c, &listObj[i].entryPoint[j].c, sizeof(obs[N-1].c));
			#if DEBUG
            	printf("Current objective is i=%d with EP j=%d, A(N-1) : x=%f & y=%f, pos current : x=%f & y=%f\n", i, j, obs[N-1].c.x, obs[N-1].c.y,_current_pos.x, _current_pos.y);
			#endif
            updateEndTraj(listObj[i].entryPoint[j].angleEP, &listObj[i].entryPoint[j].c, listObj[i].entryPoint[j].radiusEP); //plot the circle to arrive on objective

            if((g=test_in_obs(&_current_pos))!=0 ) { //Projection if the robot is inside a obstacle or "circle of end trajectory"
                project_point(_current_pos.x, _current_pos.y, obs[g].r, obs[g].c.x, obs[g].c.y, &obs[0].c);
                if( (m=test_in_obs(&obs[0].c)) !=0 ){ //Cas la projection se retrouve dans un obstacle après la premier projection
                	printf("Fix this projection dans l'obstacle n%d\n", m); //FIXME or no : investigate
                	continue;
                	}
				#if DEBUG
					printf("pos current after projection : x=%f, y=%f, obs x=%f, y=%f et r=%f\n",_current_pos.x, _current_pos.y, obs[g].c.x, obs[g].c.y, obs[g].r);
				#endif
            	}
            //déactivation d'un obstacle mobile si celui ci gene un point d'entré
            num_obs = test_in_obs(&obs[N-1].c);
            if( (num_obs >= START_FEU) && (num_obs <= (START_FEU + 16)) ){
                obs[num_obs ].active = 0;
                tabFeux[num_obs - START_FEU] = 1;
                }


            fill_tgts_lnk(); //TODO optimisation car uniquement la position de fin change dans la boucle
            a_star(A(0), A(N-1), &path_loc);

            for(m = 0 ; m < 16 ; m++){
                if(tabFeux[m] == 1){
                    obs[START_FEU + m].active = 1;
                    tabFeux[m] = 0;
                    }
                }

        	obs[0].c=_current_pos; //to be sure to have the real position if there was a projection just before

			#if DEBUG
            	printf("Pour obj=%i avec pa=%i dist=%f\n",i,j,path_loc.dist);
			#endif
            if(path_loc.dist==0) printf("ATTENTION : A* retourne distance nul\n");
            if(dist>path_loc.dist && path_loc.dist!=0) //tri distance optimal en fonction des acces
                {
                listObj[i].dist=path_loc.dist;
                dist=listObj[i].dist;
                memcpy(&path_loc2, &path_loc, sizeof(path_loc));
                memcpy(&pointactuel,&obs[N-1].c,sizeof(pointactuel));
                pt1t=obs[N-2].c;
                pt2t=obs[N-3].c;
                pt3t=obs[N-4].c;
                rt=obs[N-2].r; //same than obs[N-3] or obs[N-4]
                k++;
                }
            }

        if(k==0){
        	listObj[i].dist=0;
			#if DEBUG
				printf("No find path to achieve the objective for objective n°%d\n\n",i);
			#endif
        	continue;
        	}

        k=0;         //reset parameter for the next
        dist=999;

        tmp_val2=val_obj(i);
		#if DEBUG
        	printf("objectif n°%hhi avec ratio=%f \n\n",i,tmp_val2);
		#endif

        if (tmp_val2 > tmp_val){         //Update best objective
            tmp_val = tmp_val2;
            tmp_inx = i;
            memcpy(&path, &path_loc2, sizeof(path));
            obs[N-1].c.x = pointactuel.x;
            obs[N-1].c.y = pointactuel.y;
            pt_select.x = obs[N-1].c.x;
            pt_select.y = obs[N-1].c.y;
            pt1=pt1t;
            pt2=pt2t;
            pt3=pt3t;
            r=rt;
            }
        }

    if(tmp_inx >= 0){ //Update end of trajectory
		obs[N-2].c=pt1;
		obs[N-2].r=r;
		obs_updated[N-2]++;
		obs[N-3].c=pt2;
		obs[N-3].r=r;
		obs_updated[N-3]++;
		obs[N-4].c=pt3;
		obs[N-4].r=r;
		obs_updated[N-4]++;
    	}
	#if DEBUG
		printListObj();
        obs[N-1].c.x = pt_select.x ; //for viewing in monitoring
        obs[N-1].c.y = pt_select.y ;
        obs_updated[N-1]++;
		printf("Objectif sélectionné : %i\n\n",tmp_inx);
	#endif

    return (tmp_inx);
    }


int same_obs (sObs_t *obs1, sObs_t *obs2){
	printf("r1=%f r2=%f\n",obs1->r,obs2->r);
    return ( obs1->r == obs2->r && obs1->c.x == obs2->c.x && obs1->c.y == obs2->c.y);
	}

int same_traj (sPath_t *traj1, sPath_t *traj2) {
    unsigned int t1_ind = traj1->path_len;
    unsigned int t2_ind = traj2->path_len;

    if(traj1->path_len==0 || traj2->path_len==0)
    	return 0;
        printf("same_t 1.0\n");

    while ((int)t1_ind > 0 &&  (int)t2_ind > 0) { //pb si un step est terminer
        printf("same_t 2.0\n");
        if (same_obs (&(traj1->path[t1_ind-1].obs), &(traj2->path[t2_ind-1].obs)) ){
			t1_ind--;
			t2_ind--;
			}
        else return 0;
    }
    printf("same_t 3.0\n");
    if (!(same_obs (&(traj1->path[t1_ind].obs), &(traj2->path[t2_ind].obs))))
        return 0;

    if ((traj1->path[t1_ind].p2.x == traj2->path[t2_ind].p2.x) && (traj1->path[t1_ind].p2.y == traj2->path[t2_ind].p2.y))
        return 0;
    else
    printf("same_t 4.0\n");
    return 1 ;
}

int checkCurrentPath(void){
	static sPath_t prev_path = {.path = NULL, .path_len=0};
	int ret=-1;

	//getchar();

	printf("len1=%d et lan2=%d\n",path.path_len, prev_path.path_len);
	ret=same_traj (&path, &prev_path);
	prev_path=path;

	return ret;
	}


void set_traj(sPath_t *p, iABObs_t l[], int nb){
    int i;
    p->path = (sTrajEl_t *)calloc(nb - 1, sizeof(sTrajEl_t));
    p->path_len = nb - 1;
    // p->dist // TODO
    // p->path_len // TODO

    for(i = 0; i < nb - 1; i++){
        sSeg_t *s = tgt(l[i], l[i+1]);

        p->path[i].p1 = s->p1;
        p->path[i].p2 = s->p2;
        distPt2Pt(&s->p1, &s->p2, &p->path[i].seg_len);

        p->path[i].obs.active = 1;
        p->path[i].obs.c = obs[O(l[i+1])].c;
        p->path[i].obs.moved = 1;
        p->path[i].obs.r = fabs(obs[O(l[i+1])].r)*(1 - 2*DIR(l[i+1]));
        if(i == nb - 1){
            p->path[i].arc_len = 0.;
        }
        else{
            p->path[i].arc_len = arc_len(l[i], l[i+1], l[i+2]);
        }


        p->path[i].sid = i;
    }
}


void obj_step(){
	int i, j;
	int obj=-1;
    sObs_t obsRed[] = {
        // robots
        {{0., 0.}, 0., 1, 1},   //primary
        {{0., 0.}, R_ROBOT+12., 1, 1, 1},   //secondary
        {{0., 0.}, R_ROBOT+20., 1, 1, 1},   //primary adv
        {{0., 0.}, R_ROBOT+15., 1, 1, 1},   //secondary adv

        // trajectory
        {{ 15., 120.}, 10, 0, 1, 1}, // 4
        {{ 35., 50. }, 10, 0, 1, 1},
        {{ 50., 35. }, 10, 0, 1, 1},
        {{250., 35. }, 10, 0, 1, 1},
        {{265., 50. }, 10, 0, 1, 1},//8
        {{265., 95. }, 10, 0, 1, 1},
        {{245., 160.},10, 0, 1, 1},
        {{235., 200.}, 0, 0, 1, 1},//11
        };
    iABObs_t obs_list_Red[] = {
        A(0), // r=0
        A(4),
        B(5),
        B(6),
        B(7),
        B(8),
        B(9),
        A(10),
        A(11)// r=0
        };

    sObs_t obsYellow[] = {
        // robots
        {{0., 0.}, 0., 1, 1},   //primary
        {{0., 0.}, R_ROBOT+12., 1, 1, 1},   //secondary
        {{0., 0.}, R_ROBOT+20., 1, 1, 1},   //primary adv
        {{0., 0.}, R_ROBOT+15., 1, 1, 1},   //secondary adv

        // trajectory
        {{300. - 15., 120.}, 10, 0, 1, 1}, // 4
        {{300. - 35., 50. }, 10, 0, 1, 1},
        {{300. - 50., 35. }, 10, 0, 1, 1},
        {{300. -250., 35. }, 10, 0, 1, 1},
        {{300. -265., 50. }, 10, 0, 1, 1},//8
        {{300. -265., 95. }, 10, 0, 1, 1},
        {{300. -245., 160.},10, 0, 1, 1},
        {{300. -235., 200.}, 0, 0, 1, 1},//11
        };
    iABObs_t obs_list_Yellow[] = {
        A(0), // r=0
        B(4),
        A(5),
        A(6),
        A(7),
        A(8),
        A(9),
        B(10),
        A(11)// r=0
    };

    static iABObs_t obs_list[32];

    /*sTrajEl_t tabStart[2]={ //Segment for push a vertical fire
        {{0.  ,  0.},{10. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 0.},
        {{10 ,  0.},{10. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 1.}
        };
    sTrajEl_t tabStart[2]={ //Segment for push a vertical fire
        {{0.  ,  0.},{-10. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 0.},
        {{-10 ,  0.},{-10. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 1.}
        };
    sPath_t path;
    sMsg msgOut;

*/
    switch (state) {
    case COLOR_SELECTION:
        if(test_tirette()){
            sMsg msgOut = {{0}};
            int ret;
#if SIMU
            color = COLOR_SIMU ;
#endif
            sendPosServo(SERVO_PRIM_ARM_RIGHT, -1, 180);
            sendPosServo(SERVO_PRIM_ARM_LEFT, -1, 0);
            sendPosServo(SERVO_PRIM_DOOR, -1, 700);
            state = INIT;
            //Setting initial position
            if(color==1){
                #if PROG_TRAJ
                memcpy(obs, obsYellow, sizeof(obsYellow));
                memcpy(obs_list, obs_list_Yellow, sizeof(obs_list_Yellow));
                #endif
                obs[0].c.x = 300. - 15.5;
                obs[0].c.y = 200. - 20; //15.8
                theta_robot = M_PI;
                }
            else{
                #if PROG_TRAJ
                memcpy(obs, obsRed, sizeof(obsRed));
                memcpy(obs_list, obs_list_Red, sizeof(obs_list_Red));
                #endif
                obs[0].c.x = 15.5;
                obs[0].c.y = 200. - 20; //15.8
                theta_robot = 0;
                }

            _current_pos=obs[0].c;

            msgOut.header.type = E_POS;
            msgOut.header.size = sizeof(msgOut.payload.pos);

            msgOut.payload.pos.id = 0;
            msgOut.payload.pos.u_a = 0;
            msgOut.payload.pos.u_a_theta = 0;
            msgOut.payload.pos.u_b = 0;
            msgOut.payload.pos.theta = theta_robot;
            msgOut.payload.pos.x = obs[0].c.x;
            msgOut.payload.pos.y = obs[0].c.y;
            printf("Sending initial position to robot%i (%.2fcm,%.2fcm,%.2f°).\n", msgOut.payload.pos.id, msgOut.payload.pos.x, msgOut.payload.pos.y, msgOut.payload.pos.theta*180./M_PI);

            ret = role_send(&msgOut);
            if(ret <= 0){
                printf("bn_sendAck(E_POS) error #%i\n", -ret);
                getchar();
                }

            }

        startColor();
        break;

    case INIT :
        if(initTraj() == 1) {
            state = WAIT_STARTING_CORD;
            //Initialization of the game
            init_ele();
            //Change element for simulation
            listObj[0].utype.tree.eFruit[0]=2;
            listObj[0].utype.tree.eFruit[3]=2;
            listObj[1].utype.tree.eFruit[0]=2;
            listObj[1].utype.tree.eFruit[3]=2;
            listObj[2].utype.tree.eFruit[0]=2;
            listObj[2].utype.tree.eFruit[3]=2;
            listObj[3].utype.tree.eFruit[0]=2;
            listObj[3].utype.tree.eFruit[3]=2;
            }
        break;

    case WAIT_STARTING_CORD:
#if SIMU
        state = WAIT;
#else
        if(!test_tirette()){
            state = WAIT;
            }
#endif
        break;
    case WAIT:
        //                printf("Attente. time = %ld\n", millis()); // FIXME debug
        if(test_tirette()){
        	state = JEU;
        	_start_time = millis();
        	last_time=_start_time;

#if PROG_TRAJ
            fill_tgts_lnk();

            set_traj(&curr_path, obs_list, 9);
            curr_path.tid = ++last_tid;
            curr_traj_extract_sid = 0;
#endif

        	}
        break;
   /* case INIT_POS:
        if(mode_switch == 1){
            start = 1;
            path.path=&tabStart[0];
            path.path_len=2;
            send_robot(path) ;
            }

        if(test_tirette() && start == 1){
            state = JEU;
            _start_time = millis();
            last_time=_start_time;

            msgOut.header.type = E_POS;
            msgOut.header.size = sizeof(msgOut.payload.pos);

            msgOut.payload.pos.id = 0;
            msgOut.payload.pos.theta = theta_robot;
            msgOut.payload.pos.u_a = 0;
            msgOut.payload.pos.u_a_theta = 0;
            msgOut.payload.pos.u_b = 0;

            if(color==1){
                msgOut.payload.pos.x = 300. - 18.;
                msgOut.payload.pos.y = 200. - 13.;
                }
            else{
                msgOut.payload.pos.x = 18.;
                msgOut.payload.pos.y = 200. - 13.;
                }
            if(role_sendAck(&msgOut) > 0){
                state = JEU ;

                }
            printf("Sending initial position to robot%i (%.2fcm,%.2fcm,%.2f°).\n", msgOut.payload.pos.id, msgOut.payload.pos.x, msgOut.payload.pos.y, msgOut.payload.pos.theta*180./M_PI);
            }

        break;
*/
    case JEU :
        if(millis()-_start_time > END_MATCH) state = SHUT_DOWN;

#if PROG_TRAJ
            sGenericStatus *stPr = getLastPGStatus(ELT_PRIMARY); sPt_t ptPr;
            sGenericStatus *stAPr = getLastPGStatus(ELT_ADV_PRIMARY); sPt_t ptAPr;
            sGenericStatus *stASc = getLastPGStatus(ELT_ADV_SECONDARY); sPt_t ptASc;
            sNum_t d, dot;
            sVec_t v1, v2;

            if(stPr){
                ptPr.x = stPr->prop_status.pos.x;
                ptPr.y = stPr->prop_status.pos.y;

                if(stAPr){
                    ptAPr.x = stAPr->prop_status.pos.x;
                    ptAPr.y = stAPr->prop_status.pos.y;

                    distPt2Pt(&ptPr, &ptAPr, &d);
                    v1.x = cos(stPr->prop_status.pos.theta);
                    v1.y = sin(stPr->prop_status.pos.theta);
                    convPts2Vec(&ptPr, &ptAPr, &v2);
                    dotVecs(&v1, &v2, &dot);

                    if(d < 50 && dot > 0.6*d){
// TODO
                        printf("CONTACT PRIM!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
                    }
                }

                if(stASc){
                    ptASc.x = stASc->prop_status.pos.x;
                    ptASc.y = stASc->prop_status.pos.y;

                    distPt2Pt(&ptPr, &ptASc, &d);
                    v1.x = cos(stPr->prop_status.pos.theta);
                    v1.y = sin(stPr->prop_status.pos.theta);
                    convPts2Vec(&ptPr, &ptASc, &v2);
                    dotVecs(&v1, &v2, &dot);

                    if(d < 40 && dot > 0.6*d){
// TODO
                        printf("CONTACT SEC!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
                    }
                }
            }
#else
        //Test si tous objectif sont fini, temporaire pour eviter spam à la fin
        temp=0;
        for(j=0 ; j<NB_OBJ ; j++){
            if(listObj[j].active==0) temp++;
        	}
        if(temp==NB_OBJ) state = SHUT_DOWN;

        //Calculation of the next objective
        if( (((millis()-last_time)>1000) && (mode_obj==0))){
            printf("obs[0] suivi par next_obj(): x=%f & y=%f\n", obs[0].c.x, obs[0].c.y);
            last_time=millis();

            if( (obj = next_obj()) != -1 ){
            	current_obj=obj;
				if ( checkCurrentPath() == 0 || checkRobotBlock() == 1){
					send_robot(path) ;
					}
				}
			}


        //Update position
            if( (millis() - _start_time) > 2000){
               // simuSecondary();
                }
            posPrimary();

            checkRobot2Obj();
            checkRobotBlock();


       if((millis()-last_time2)>1000){
            last_time2 = millis();
            updateEntryPointTree();
            printf("Position actuel : x=%f et y=%f\n", _current_pos.x,_current_pos.y);
            printf("Select : x=%f et y=%f avec fabsx=%f et fabsy=%f\n", pt_select.x,pt_select.y, fabs(pt_select.x-_current_pos.x),fabs(pt_select.y-_current_pos.y));
            }

#endif

        //If the select point is achieved
        if (((fabs(pt_select.x-_current_pos.x)<RESO_POS && fabs(pt_select.y-_current_pos.y)<RESO_POS) ) || mode_obj==1){   //objectif atteint
        	//printf("(listObj[current_obj]).type=%d et curent_obj=%d, mode_obj=%d\n",(listObj[current_obj]).type, current_obj, mode_obj);
        	//printf("Select : x=%f et y=%f avec fabsx=%f et fabsy=%f\n", pt_select.x,pt_select.y, fabs(pt_select.x-_current_pos.x),fabs(pt_select.y-_current_pos.y));
        	//printf("mode_obj=%d", mode_obj);

        	switch ((listObj[current_obj]).etype){ //Mise en place des procédures local en fonction de l'objectif
				case E_ARBRE :
					obj_tree(current_obj);
					break;
				case E_BAC :
					obj_bac(current_obj);
					break;
				case E_FEU :
					obj_fire(current_obj);
					break;
				default:
					printf("Erreur current_obj\n");
					getchar();
					break;
				}
        	}

        objBonusFire();


        break;

    case SHUT_DOWN:
        printf ("SHUT_DOWN : time = %d\n", (int) (millis()-_start_time)/1000);
        exit(1);
        //TODO arrêt total
        return;
        break;
    default:
        break;
		}
	}

int obj_init(){
	int i, ret, state = 0;

    if(sizeof(obs)/sizeof(*obs) != N){
        printf("N isn't correct, byebye\n");
        exit(1);
    	}
/*
    //ping all address
#if !SIMU
    while(1){
        switch(state){
            case 0:
                if( (ret = bn_ping(ADDRU2_MAIN_PROP)) > 0){
                    state = 1;
                    }
                printf("Ping main prop : %d\n", ret);
                break;
            case 1:
                if( (ret = bn_ping(ADDRU1_MAIN_IO)) > 0){
                    state = 2;
                    }
                printf("Ping main io : %d\n", ret);
                break;
            case 2:
                if( (ret = bn_ping(ADDRX_MOBILE_1)) > 0){
                    state = 3;
                    }
                printf("Ping mobile 1 : %d\n", ret);
                break;
            case 3:
                if( (ret = bn_ping(ADDRX_MOBILE_2)) > 0){
                    state = 5; // XXX
                    }
                printf("Ping mobile 2 : %d\n", ret);
                break;
//            case 4:
//                if( (ret = bn_ping(ADDRX_FIX)) > 0){
//                    state = 5;
//                    }
//                printf("Ping fix : %d\n", ret);
//                break;

            }
        if(state == 5) break;
        }
#endif
*/
#if !PROG_TRAJ
    listObj[8].done=0.5;
    listObj[10].done=0.5;
#endif
   // starting_cord = 1; //tirette simulation
    mode_switch = 0;


    //Update all obstacle
    for(i=0;i<N;i++){
        obs_updated[i]++;
    	}

#if !PROG_TRAJ
	#if DEBUG
		printListObj();
	#endif
#endif

    return 0;
	}
