#include <time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

#include "millis.h"

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
sPt_t pt_select;

int mode_obj=0;



void update_end (sNum_t radius,  sPt_t *center)
	{
    obs[N-1].c = *center;
    obs[N-1].r = radius;
    obs[N-1].active = 1;
	}

void active (sObs_t *obs) {obs->active = 1;}
void unactive (sObs_t *obs) {obs->active = 0;}



sNum_t val_obj(int num) //numeros de l'objectif dans listObj[] compris entre 0 et NB_OBJ
	{
	#if DEBUG
		printf("Debut val_obj avec num=%i\n",num);
	#endif

	sNum_t speed=1; // in m/s //TODO Faire pour trajectoire circulaire
	sNum_t time;
	sNum_t dist;
	sNum_t point;
	sNum_t ratio;

	dist=listObj[num].dist;
	if(dist==0)
		{
		printf("Attention : distance nul\n"); //Erreur ou objectif atteint
		return (-1);
		}

	time=dist/speed;
	if(time > (END_MATCH-(millis()-_start_time))) //temps restant insuffisant
		{
		listObj[num].active=0;
		return 0;
		}

    switch(listObj[num].type)
    	{
		case E_FEU :
			break;

		case E_TORCHE_MOBILE :
			break;

        case E_ARBRE :
			#if DEBUG
				printf("enum=%i => type=E_ARBRE, dist=%f\n",listObj[num].type,dist);
			#endif
        	point=((Obj_arbre*)listObj[num].typeStruct)->nb_point;
        	ratio=point/time*1000+ratio_arbre() ;
        	printf("ratio_arbre()=%f et n=%d\n",ratio_arbre(),num);
        	break;

        case E_BAC :
			#if DEBUG
				printf("enum=%i => type=E_BAC, dist=%f\n",listObj[num].type,dist);
			#endif
			point=((Obj_bac*)listObj[num].typeStruct)->nb_point;
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
	printf("Debut next_obj\n");

    sNum_t tmp_val = 0.;
    sPath_t path_loc={.dist = 0.,  .path = NULL };
    sPath_t path_loc2={.dist = 0.,  .path = NULL };
    sNum_t tmp_val2;
    sNum_t dist=999.;
    sPt_t pointactuel;

    int i,j, tmp_inx=-1; //tmp_inx : index de l'objectif qui va  etre selectionner


    for(i = 0 ; i < NB_OBJ ; i++)
    	{
        if (listObj[i].active==0) continue; //test si ojectif existe encore

        for(j=0 ; j<listObj[i].nbObs ; j++) obs[listObj[i].listIABObs[j]].active=0;

        for(j=0 ; j<listObj[i].nbEP ; j++)
			{
			if(listObj[i].entryPoint[j].active==0) continue;
			memcpy(&obs[N-1], &listObj[i].entryPoint[j], sizeof(obs[N-1]));
			printf("Next obj AN-1 x=%f & y=%f\n", obs[N-1].c.x, obs[N-1].c.y);
			fill_tgts_lnk(); //TODO optimisation car uniquement la position de fin change dans la boucle
	    	a_star(A(0), A(N-1), &path_loc);
	    	printf("Pour obj=%i avec pa=%i dist=%f\n",i,j,path_loc.dist);
	    	if(path_loc.dist==0) printf("\nATTENTION : Erreur\n\n");
	    	if(dist>path_loc.dist)
				{
	    		listObj[i].dist=path_loc.dist;
				dist=listObj[i].dist;
				path_loc2=path_loc;
				pointactuel.x=obs[N-1].c.x;
				pointactuel.y=obs[N-1].c.y;
				}
			}
        tmp_val2=val_obj(i);
        dist=999;

        for(j=0 ; j<listObj[i].nbObs ; j++) obs[listObj[i].listIABObs[j]].active=1;

		printf("objectif n°%hhi avec ratio=%f \n\n",i,tmp_val2);

		//Actualisation meilleur objectif
		if (tmp_val2 > tmp_val)
			{
			tmp_val = tmp_val2;
			tmp_inx = i;
			memcpy(&path, &path_loc2, sizeof(path));
			obs[N-1].c.x = pointactuel.x;
			obs[N-1].c.y = pointactuel.y;
			pt_select.x = obs[N-1].c.x;
			pt_select.y = obs[N-1].c.y;
			}
    	}

	printListObj();
	printf("Objectif sélectionné : %i\n\n",tmp_inx);
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

int test_tirette()
	{
	return(1);
	}

void obj_tree(iABObs_t obj)
	{
	int static first=0;

	if(first==0)
		{
		printf("Debut objectif arbre\:");
		listObj[obj].active=0;
		bac.nb_point=bac.nb_point+3; //TODO peut etre 4 dépend ou est le fruit noir
		mode_obj=1;

		if((pt_select.x==listObj[obj].entryPoint[0].c.x) && (pt_select.y==listObj[obj].entryPoint[0].c.y))
			{
			pt_select=(listObj[obj].entryPoint[1]).c;
			printf("Sortie 2 selectionné\n");
			}
		else
			{
			pt_select=(listObj[obj].entryPoint[0]).c;
			printf("Sortie 1 selectionné\n");
			}

		obs[N-1].c.x=pt_select.x;
		obs[N-1].c.y=pt_select.y;

		obs[obj+1].r=R_ROBOT + 5;

		first=1;
		printf("Objectif : arbre n°%hhi en cour\n\n\n",obj);
		}
	else
		{
		if( (millis()-last_time)>1000)
			{
			last_time=millis();
			fill_tgts_lnk();
			a_star(A(0), A(N-1), &path);
			send_robot(path) ;
			}
		if ((fabs(pt_select.x-_current_pos.x)<RESO_POS && fabs(pt_select.y-_current_pos.y)<RESO_POS))
			{
			mode_obj=0;
			first=0;
			pt_select.x=0;
			pt_select.y=0;
			}
		}
	}

void obj_bac(iABObs_t obj)
	{
	listObj[obj].active=0;
	bac.nb_point=0;
	#if DEBUG
		printf("Objectif : bac fini\n\n\n");
	#endif
	}
void obj_tree2(iABObs_t obj)
	{
	listObj[obj].active=0;
	#if DEBUG
		printf("Objectif : arbre fondn°%hhi fini\n\n\n",obj);
	#endif
	}


int test_in_obs() //retourne le numéros de l'obstable si la position est a l'interieur de celui ci
	{		//FIXME si le robot dans plusieur obstable
	int i;
	for(i=1; i<N-2;i++)
		{
		if(obs[i].active==0)continue;
		if( sqrt(pow(obs[i].c.x-_current_pos.x,2)+pow(obs[i].c.y-_current_pos.y,2)) < obs[i].r)
			{
			printf("Le robot est dans l'obstacle n=%i\n",i);
			return i;
			}
		}
	return 0;
	}

void state_machine()
	{
    gat_ind = 1;
    estate_t state = ATTENTE;
    iABObs_t current_obj=0;
    iABObs_t current_obs;
    sPt_t point;
    int i,j;
   // sPath_t path;
    //next_path.tid = 1;
   // sPath_t current_path
   // sPath_t *_current_path = &current_path; //non utiliser pour le moment
  //  sPath_t *_next_path = &next_path;


    while(1){

        switch (state) {

            case ATTENTE :
//                printf("Attente. time = %ld\n", millis()); // FIXME debug
				if ( test_tirette() )
					{
					state = JEU;
					_start_time = millis();
					last_time=_start_time;
					}
        		break;

            case JEU :
            	if (millis()-_start_time > 90000) state = SHUT_DOWN;

				if( ((millis()-last_time)>1000) && (mode_obj==0)) //calcul objectif suivant toute les seconde
					{
					printf("obs[0] suivi par next_obj(): x=%f & y=%f\n", obs[0].c.x, obs[0].c.y);
					current_obj = next_obj();
					//getchar();
					if ( current_obj != -1)  //aucun objectif actif atteignable si -1
						{
						send_robot(path) ;// Envoie au bas niveau la trajectoire courante
						last_time=millis();
						}
					}

				if(get_position(&_current_pos))
					{

					//printf("Position actuel avant correction : x=%f et y=%f\n", _current_pos.x,_current_pos.y);
					//for(j=0 ; j<listObj[current_obj].nbObs;j++) obs[(listObj[current_obj].listIABObs[j])].active=0;
					//if((i=test_in_obs())!=0) project_point(_current_pos.x, _current_pos.y, obs[i].r, obs[i].c.x,obs[i].c.y, &_current_pos);;
					//for(j=0 ; j<listObj[current_obj].nbObs;j++) obs[(listObj[current_obj].listIABObs[j])].active=1;

					memcpy(&obs[0].c,&_current_pos, sizeof(obs[0].c));
					if((millis()-last_time2)>1000)
						{
						long last_time2=0;
						printf("Position actuel : x=%f et y=%f\n", _current_pos.x,_current_pos.y);
						printf("Select : x=%f et y=%f avec fabsx=%f et fabsy=%f\n", pt_select.x,pt_select.y, fabs(pt_select.x-_current_pos.x),fabs(pt_select.y-_current_pos.y));
						}
					}
				if ((fabs(pt_select.x-_current_pos.x)<RESO_POS && fabs(pt_select.y-_current_pos.y)<RESO_POS) || mode_obj==1)   //objectif atteint
					{
					switch ((listObj[current_obj]).type) //Mise en place des procedure local en fonction de l'objectif
						{
						case E_ARBRE :
							obj_tree(current_obj);
							break;
						case E_BAC :
							obj_bac(current_obj);
							state = SHUT_DOWN; //temporaire
							break;
						}
					}

                break;

            case SHUT_DOWN:
            printf ("SHUT_DOWN\n");
                //TODO arrêt total
            	return;
            	break;


              }
        }
    return;
    }


int main()
	{
	//Position de départ
	obs[0].c.x=1;
	obs[0].c.y=199;
	_current_pos=obs[0].c;

	init_mes(); //initialisation de la communication
	init_ele();	//initialisation des éléments

	printListObj();

	/*//test

//	for(i=1;i<N-1;i++) obs[i].active=0;
	obs[N-1].c.x=20;
	obs[N-1].c.y=40;
	 last_time=millis();

	while(1)
	{
	fill_tgts_lnk();


    for(i=0; i<2*N; i++) {
        printf(" %u%c", (i>>1)%10, obs[i>>1].r?(i&1?'b':'a'):' ');
    }
    printf("\n");
    for(i=0; i<2*N; i++) {
        printf("%u%c", (i>>1)%10, obs[i>>1].r?(i&1?'b':'a'):' ');
        for(j=0; j<2*N; j++) {
            printf("%c%c ", j?',':' ', lnk[i][j]?'1':' ');
        }
        printf("\n");
    }


	a_star(A(0), A(N-1), &path);
	printf("dist=%f\n",path.dist);

	 if((millis()-last_time)>1000)
		 {
		 send_robot(path) ;
		 last_time=millis();
		 }
	get_position(&_current_pos);
	obs[0].c=_current_pos;
	}
*/

	/*// TEST prog
      	if(first==0)
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

	state_machine();

    return 0;
}

