#include "obj.h"


#define CLAMP(m, v, M) MAX(m, MIN(v, M))


estate_t state = ATTENTE;
int temp=0; //Temporaire pour mettre en shutdown lorsque tous les objectifs sont finis
int current_obj=-1;
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
        listObj[num].active=0;
        return 0;
        }

    switch(listObj[num].type){
        case E_FEU :
            printf("enum=%i => type=E_FEU, dist=%f\n",listObj[num].type,dist);
            point=((Obj_feu*)listObj[num].typeStruct)->nb_point;
            ratio=point/time*100;
            break;

        case E_TORCHE_MOBILE :
            break;

        case E_ARBRE :
            printf("enum=%i => type=E_ARBRE, dist=%f\n",listObj[num].type,dist);
            point=((Obj_arbre*)listObj[num].typeStruct)->nb_point;
            ratio=point/time*1000+ratio_arbre() ;
            printf("ratio_arbre()=%f et n=%d\n",ratio_arbre(),num);
            break;

        case E_BAC :
            printf("enum=%i => type=E_BAC, dist=%f\n",listObj[num].type,dist);
            point=((Obj_bac*)listObj[num].typeStruct)->nb_point;
            ratio=point/time*1000+ratio_bac();
            break;

        case E_FOYER :
            break;

        case E_TORCHE_FIXE :
            break;

        default:
        	return(-1);
        	break;
        }
    return ratio;
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

            if((g=test_in_obs())!=0 ) { //Projection if the robot is inside a obstacle or "circle of end trajectory"
                project_point(_current_pos.x, _current_pos.y, obs[g].r, obs[g].c.x, obs[g].c.y, &obs[0].c);
                if(test_in_obs()!=0 ){ //Cas la projection se retrouve dans un obstacle après la premier projection
                	printf("Fix this projection dans un obstacle\n"); //FIXME or no : investigate
                	continue;
                	}
				#if DEBUG
					printf("pos current after projection : x=%f, y=%f, obs x=%f, y=%f et r=%f\n",_current_pos.x, _current_pos.y, obs[g].c.x, obs[g].c.y, obs[g].r);
				#endif
            	}

            fill_tgts_lnk(); //TODO optimisation car uniquement la position de fin change dans la boucle
            a_star(A(0), A(N-1), &path_loc);

        	_current_pos=obs[0].c; //to be sure to have the real position if there was a projection just before

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
				printf("No find path to achieve the objectivefor objective n°%d\n\n",i);
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



void obj_step(){
	int i,j;
    switch (state) {
    case ATTENTE :
        //                printf("Attente. time = %ld\n", millis()); // FIXME debug
        if(test_tirette()){
        	state = JEU;
            _start_time = millis();
            last_time=_start_time;
        	}
        break;

    case JEU :
        if(millis()-_start_time > END_MATCH) state = SHUT_DOWN;

        //Test si tous objectif sont fini, temporaire pour eviter spam à la fin
        temp=0;
        for(j=0 ; j<NB_OBJ ; j++){
            if(listObj[j].active==0) temp++;
        	}
        if(temp==NB_OBJ) state = SHUT_DOWN;

        //Calculation of the next objective
        if( (((millis()-last_time)>1000) && (mode_obj==0))){
            printf("obs[0] suivi par next_obj(): x=%f & y=%f\n", obs[0].c.x, obs[0].c.y);
            current_obj = next_obj();
            if ( current_obj >= 0){  	//aucun objectif actif atteignable si -1
                send_robot(path) ;		// Envoie au bas niveau la trajectoire courante
                last_time=millis();
            	}
        	}

        //Update position
        if(get_position(&_current_pos)){ //TODO A verifier
            memcpy(&obs[0].c,&_current_pos, sizeof(obs[0].c));
            obs_updated[0]++;

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

            if((millis()-last_time2)>1000){
                last_time2 = millis();
                updateEntryPointTree();
                printf("Position actuel : x=%f et y=%f\n", _current_pos.x,_current_pos.y);
                printf("Select : x=%f et y=%f avec fabsx=%f et fabsy=%f\n", pt_select.x,pt_select.y, fabs(pt_select.x-_current_pos.x),fabs(pt_select.y-_current_pos.y));
            	}
        	}

        //If the select point is achieved
        if ((fabs(pt_select.x-_current_pos.x)<RESO_POS && fabs(pt_select.y-_current_pos.y)<RESO_POS) || mode_obj==1){   //objectif atteint
            switch ((listObj[current_obj]).type){ //Mise en place des procedure local en fonction de l'objectif
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
					break;
				}
        	}
        break;

    case SHUT_DOWN:
        printf ("SHUT_DOWN\n");
        exit(1);
        //TODO arrêt total
        return;
        break;
		}
	}

int obj_init(){
	int i;
    if(sizeof(obs)/sizeof(*obs) != N){
        printf("N isn't correct, byebye\n");
        exit(1);
    	}

    //Setting initial position
    if(COLOR==1){
        obs[0].c.x=300. - 16.;
        obs[0].c.y=200. - 16.;
        }
    else{
        obs[0].c.x=16.;
        obs[0].c.y=200. - 16.;
        }
    _current_pos=obs[0].c;

    //Initialization of the game
    init_ele();

    //Add bad fruit for simulation
    ((Obj_arbre*)listObj[0].typeStruct)->eFruit[3]=2;
    ((Obj_arbre*)listObj[1].typeStruct)->eFruit[0]=2;
    ((Obj_arbre*)listObj[2].typeStruct)->eFruit[3]=2;
    ((Obj_arbre*)listObj[3].typeStruct)->eFruit[4]=2;


    //Update all obstacle
    for(i=0;i<N;i++){
        obs_updated[i]++;
    	}

	#if DEBUG
		printListObj();
	#endif

    return 0;
	}