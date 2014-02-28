#include <time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../../global_errors.h"
#include "node_cfg.h"

#include "a_star.h"
#include "math_types.h"
#include "math_ops.h"
#include "tools.h"
#include "type_ia.h"

#define CLAMP(m, v, M) MAX(m, MIN(v, M))

// constantes qui vont bien TODO TODO TODO !!!!!!
sNum_t t_lastg_l = 0.;   // time_lastglass left
sNum_t t_lastg_r = 1000.;
sNum_t av_occ_t = 2000.;    // average occupation time
int gnb_l,gnb_r; // glass number right/left
sNum_t pts_per_msec = 128./90000.;
int gat_ind;

sMsg inMsg, outMsg;



void update_end (sNum_t radius,  sPt_t *center)
	{
    obs[N-1].c = *center;
    obs[N-1].r = radius;
    obs[N-1].active = 1;
	}

void active (sObs_t *obs) {obs->active = 1;}
void unactive (sObs_t *obs) {obs->active = 0;}



sNum_t val_obj(int num) //numeros de l'objectif dans list_obj[] compris entre 0 et NB_OBJ
	{
	#if DEBUG
		printf("Debut val_obj\n");
	#endif

	sNum_t speed=1; // in m/s //TODO Faire pour trajectoire circulaire
	sNum_t time;
	sNum_t dist;
	sNum_t point;
	sNum_t ratio;

	fill_tgts_lnk();
	a_star(A(0), A(N-1), &path);
	dist=path.dist;
	if(dist==0) return (-1); //On est sur l'objectif
	time=dist/speed;
	if(time > (END_MATCH-millis())) //temps restant insuffisant
		{
		list_obj[num].active=0;
		return 0;
		}

    switch(list_obj[num].type)
    	{
		case E_FEU :
			break;

		case E_TORCHE_MOBILE :
			break;

        case E_ARBRE :
			#if DEBUG
				printf("enum=%i => type=E_ARBRE, dist=%f\n",list_obj[num].type,dist);
			#endif
        	point=((Obj_arbre*)list_obj[num].type_struct)->nb_point;
        	ratio=point/time*1000+ratio_arbre() ;
        	break;

        case E_BAC :
			#if DEBUG
				printf("enum=%i => type=E_BAC, dist=%f\n",list_obj[num].type,dist);
			#endif
			point=((Obj_bac*)list_obj[num].type_struct)->nb_point;
        	ratio=point/time*1000+ratio_bac();
        	break;

        case E_FOYER :
        	break;

        case E_TORCHE_FIXE :
        	break;
    	}
    return ratio;
	}

iABObs_t next_obj (void)
	{
	#if DEBUG
		printf("Debut next_obj\n");
	#endif
    sNum_t tmp_val = 0.;
    sNum_t tmp_val2;
    sNum_t tmp_val3;
    int tmp_inx=-1; //index de l'objectif qui va  etre selectionner
    int i;
    for(i = 0 ; i < NB_OBJ ; i++)
    	{
        if (list_obj[i].active==0) continue; //test si ojectif existe encore

        switch(list_obj[i].type)
        	{
        	case E_ARBRE :
            	obs[i+1].active=0;
    			memcpy(&obs[N-1], &(((Obj_arbre*)list_obj[i].type_struct)->entrer1), sizeof(obs[N-1]));
    			tmp_val2 = val_obj(i);

    			memcpy(&obs[N-1], &(((Obj_arbre*)list_obj[i].type_struct)->entrer2), sizeof(obs[N-1]));
    			tmp_val3=val_obj(i);

    			if(tmp_val3>tmp_val2) tmp_val2=tmp_val3;
    			obs[i+1].active=1;
    			break;
        	case E_BAC :
        		obs[i+1+5].active=0;
        		obs[i+2+5].active=0;
        		obs[i+3+5].active=0;
        		memcpy(&obs[N-1], &(((Obj_bac*)list_obj[i].type_struct)->entrer), sizeof(obs[N-1]));
        		tmp_val2 =val_obj(i);
        		obs[i+1+5].active=1;
        		obs[i+2+5].active=1;
        		obs[i+3+5].active=1;
        		break;
        	}

			//determination ratio objectif
			#if DEBUG
					printf("objectif n°%hhi avec ratio=%f \n\n",i,tmp_val2);
			#endif

			//Actualisation meilleur objectif
			if (tmp_val2 > tmp_val)
				{
				tmp_val = tmp_val2;
				tmp_inx = i;
				}
			if(tmp_inx != -1)
				{
				obs[tmp_inx+1].r=R_ROBOT + 5;
				}
    	}
    return (tmp_inx);
	}

int same_obs (sObs_t *obs1, sObs_t *obs2){
    return ( obs1->r == obs2->r && obs1->c.x == obs2->c.x && obs1->c.y == obs2->c.y);

}

int same_traj (sPath_t *traj1, sPath_t *traj2) {
    unsigned int t1_ind = traj1->path_len;
    unsigned int t2_ind = traj2->path_len;
        printf("same_t 1.0\n");

    while ((int)t1_ind > 0 &&  (int)t2_ind > 0) {
        printf("same_t 2.0\n");

        if (same_obs (&(traj1->path[t1_ind].obs), &(traj2->path[t2_ind].obs)) ){
        t1_ind--; t2_ind--;
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

int test_tirette() //Simulation TODO fonction réel
	{
	return(millis() > 500); // FIXME
	}

void send_robot(iABObs_t goal)
	{
	obs[N-1].c.x=obs_PA[2*goal].c.x;
	obs[N-1].c.y=obs_PA[2*goal].c.y;
	}

void get_position(iABObs_t goal, sObs_t *pos) //TODO normalement seulement le dernier parametre
	{
	pos->c.x=obs_PA[2*goal].c.x;
	pos->c.y=obs_PA[2*goal].c.y;
	}

void obj_tree(iABObs_t obj)
	{
	sleep(5);//simule le temps de faire l'action
	list_obj[obj].active=0;
	bac.nb_point=bac.nb_point+3;
	#if DEBUG
		printf("Objectif : arbre n°%hhi fini\n\n\n",obj);
	#endif
	}

void obj_bac(iABObs_t obj)
	{
	list_obj[obj].active=0;
	bac.nb_point=0;
	#if DEBUG
		printf("Objectif : bac fini\n\n\n");
	#endif
	}

void state_machine()
	{
    gat_ind = 1;
    estate_t state = ATTENTE;
    iABObs_t current_obj;
    iABObs_t current_obs;
    int stop=10;
    int stop_int=0;

    sPath_t next_path;
    //next_path.tid = 1;
   // sPath_t current_path
   // sPath_t *_current_path = &current_path; //non utiliser pour le moment
    sPath_t *_next_path = &next_path;


    while(1){

        switch (state) {

            case ATTENTE :
//                printf("Attente. time = %ld\n", millis()); // FIXME debug
                if ( test_tirette() )
                	{
                	state = JEU;
                	_start_time = time(NULL);
                	}
        		break;

            case JEU :
                if (millis() > 90000) state = SHUT_DOWN;

                //temporaine pour test
            	stop_int++;
            	if(stop_int==stop) state = SHUT_DOWN;

				  // TEST prog
                    	/*if(first==0)
                    		{
							  sNum_t xpos, ypos; //FIXME debug
							  printf("Jeu. time = %ld\n", millis()); //FIXME debug
							  printf("position?\n");
							  fflush(stdout);
							 // scanf("%f %f", &xpos, &ypos);
							  obs[0].c.x = 10;
							  obs[0].c.y = 170;
							  printf("cx %f cy %f \n",obs[0].c.x ,obs[0].c.y);
						   //END test
							  first=1;
                    		}*/

					  current_obj = next_obj();

					  current_obs = (list_obj[current_obj]).num_obs ;//Conversion objectif -> obstable
					  if ( current_obj == -1) continue; //aucun objectif actif atteignable

					  fill_tgts_lnk();
					  a_star(A(0), current_obs, _next_path);

					  send_robot(current_obj) ;// Envoie au bas niveau la trajectoire courante
					  get_position(current_obj, &_current_pos);
					  memcpy(&obs[0],&_current_pos, sizeof(obs[0]));
					  printf("Position actuel : x=%f et y=%f\n", obs[0].c.x,obs[0].c.y);
					  if (fabs(obs[N-1].c.x -_current_pos.c.x)<RESO_POS && fabs(obs[N-1].c.y -_current_pos.c.y)<RESO_POS)   //objectif atteint
						  {
						  switch ((list_obj[current_obj]).type) //Mise en place des procedure local en fonction de l'objectif
						  	  {
						  	  case E_ARBRE :
						  		  obj_tree(current_obj);
						  		  break;
						  	  case E_BAC :
						  		  obj_bac(current_obj);
						  		  break;


							  }
						  }
                break;

            case SHUT_DOWN:
           // printf ("SHUT_DOWN\n");
                //TODO arrêt total
            	return;
            	break;


              }
        }
    return;
    }


int main()
	{
	int i, ret;
    sPath_t path;

	ret = bn_init();
	if(ret < 0){
		printf("bn_init() failed #%i\n", -ret);
		exit(1);
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
    if(ret <= 0){
        printf("bn_sendAck() error #%i\n", -ret);
    }
    fill_tgts_lnk();
    a_star(A(0), A(N-1), &path);
    printf("path from 0a to %ua (dist %.2fcm & path_len=%i):\n", N-1, path.dist, path.path_len);

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
    		outMsg.payload.traj.tid = 0;

    		ret = bn_sendAck(&outMsg);
    		if(ret < 0){
    			printf("bn_send() failed #%i\n", -ret);
    		}

    		outMsg.header.destAddr = ADDRD_DEBUG;
    		ret = bn_sendAck(&outMsg);
    		if(ret < 0){
    			printf("bn_send() failed #%i\n", -ret);
    		}

    		usleep(1000);
    	}

    while(1) {
    	ret = bn_receive(&inMsg);
    	if(ret < 0){
    		printf("bn_receive() error #%i\n", -ret);
    		continue;
    	}

    	if(ret > 0){
			switch(inMsg.header.type){
			case E_POS:
				printf("received position (%.1fcm,%.1fcm,%.2f°)\n", inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta*180./M_PI);

				memcpy(&outMsg, &inMsg, sizeof(inMsg.header)+inMsg.header.size);
				outMsg.header.destAddr = ADDRD_DEBUG;
				ret = bn_send(&outMsg);
				if(ret < 0){
					printf("bn_send() error #%i\n", -ret);
				}
				break;
			default:
				//printf("received unknown messsage, type %s (%i)\n", eType2str(inMsg.header.type), inMsg.header.type);
				break;
			}
    	}
    }

#if 0
#if SEB
	obs[0].active = 1;
    obs[2].active = 1;
    obs[3].active = 1;

    //Changement de la destination
    obs[N-1].c.x = 80;
    obs[N-1].c.y = 180;
    obs[N-1].active = 1;

#else
    //Activation des éléments du jeu
	int i, j;
	for(i=0; i<Nb_obs_start;i++ )
		{
		obs[i].active =1;
		}
	//Initialisation des arbres
		for(i=0 ; i<4 ; i++)
			{
			list_obj[i].type_struct = &arbre[list_obj[i].num_obj];
			((Obj_arbre*)list_obj[i].type_struct)->nb_point=10;
			for(j=0 ; j<6 ; j++)
				{
				((Obj_arbre*)list_obj[i].type_struct)->eFruit[j]=0;
				}
			((Obj_arbre*)list_obj[i].type_struct)->entrer1=obs_PA[2*i];
			((Obj_arbre*)list_obj[i].type_struct)->entrer2=obs_PA[2*i+1];
			}
	//Initialisation du bac
		list_obj[5].type_struct = &bac;
		((Obj_bac*)list_obj[i].type_struct)->nb_point=0;
		((Obj_bac*)list_obj[i].type_struct)->entrer=obs_PA[8];

	//TODO Initialisation des autres éléments
	//TODO activation bac jaune ou rouge
#endif



	//Initialisation de la position courante;
	_current_pos.c.x=10;
	_current_pos.c.y=170;
	_current_pos.r=0;
	_current_pos.moved=1;
	_current_pos.active=1;

#if DEBUG
	printf("Fin d'initialisation des éléments de jeu\n");
#endif

    _start_time = time(NULL);
    state_machine();
#endif

    return 0;
}

