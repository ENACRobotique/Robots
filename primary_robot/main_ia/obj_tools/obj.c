#include <time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../../global_errors.h"
#include "node_cfg.h"

#include "a_star.h"
#include "math_types.h"
#include "math_ops.h"
#include "tools.h"
#include "obj_types.h"
#include "millis.h"

#define CLAMP(m, v, M) MAX(m, MIN(v, M))


sPt_t pt_select;
int prio=0;
int part=-1;
estate_t state = ATTENTE;
int temp=0;
int current_obj=-1;
int mode_obj=0;


sNum_t val_obj(int num) //numeros de l'objectif dans listObj[] compris entre 0 et NB_OBJ
    {
    printf("Debut val_obj avec num=%i\n",num);

    sNum_t speed=1;
    sNum_t time;
    sNum_t dist;
    sNum_t point;
    sNum_t ratio = 0.;

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
            printf("enum=%i => type=E_FEU, dist=%f\n",listObj[num].type,dist);
            point=((Obj_feu*)listObj[num].typeStruct)->nb_point;
            ratio=point/time*1000;
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

int8_t next_obj (void)
    {
    printf("Debut next_obj\n");

    sNum_t tmp_val = 0.;
    sPath_t path_loc={.dist = 0.,  .path = NULL };
    sPath_t path_loc2={.dist = 0.,  .path = NULL };
    sNum_t tmp_val2;
    sNum_t dist=999.;
    sPt_t pointactuel={0,0};
    float dist_prev_current=0; //FIXME Verifier utilité
    int obs_deactive[MAX_OBS_BY_OBJ];
    int k=1;
    int l=0,m=0;
    int i,j, tmp_inx=-1; //tmp_inx : index de l'objectif qui va  etre selectionner

    obs[N-1].active=1;

    for(i = 0 ; i < NB_OBJ ; i++)
        {
        if (listObj[i].active==0) continue; //test si ojectif existe encore

        for(j=0 ; j<listObj[i].nbObs ; j++)
            {
            if(obs[listObj[i].numObs[j]].active==1)
                {
                obs[listObj[i].numObs[j]].active=0;
                obs_updated[listObj[i].numObs[j]]++;
                }
            else //permet de memoriser l'état initiale
                {
                obs_deactive[l]=j;
                l++;
                }
            }
		#if DEBUG
				printObsActive();
		#endif
        for(j=0 ; j<listObj[i].nbEP ; j++)
            {
            memcpy(&obs[N-1].c, &listObj[i].entryPoint[j].c, sizeof(obs[N-1].c));
			#if DEBUG
            	printf("Next obj A(N-1) : x=%f & y=%f\n", obs[N-1].c.x, obs[N-1].c.y);
			#endif
            fill_tgts_lnk(); //TODO optimisation car uniquement la position de fin change dans la boucle
            a_star(A(0), A(N-1), &path_loc);
			#if DEBUG
            	printf("Pour obj=%i avec pa=%i dist=%f\n",i,j,path_loc.dist);
			#endif
            if(path_loc.dist==0) printf("ATTENTION : A* retourne distance nul\n");
            if(dist>path_loc.dist)
                {
                if(i==current_obj && k==1)
                    {
                    dist_prev_current=listObj[i].dist;
                    k=0;
                    }
                listObj[i].dist=path_loc.dist;
                dist=listObj[i].dist;
                path_loc2=path_loc;
                pointactuel.x=obs[N-1].c.x;
                pointactuel.y=obs[N-1].c.y;
                }
            }
        tmp_val2=val_obj(i);
        dist=999;

        for(j=0 ; j<listObj[i].nbObs ; j++)
        	{
            obs[listObj[i].numObs[j]].active=1;
            obs_updated[listObj[i].numObs[j]]++;
        	}
        for(m=0 ; m<l ; m++) //remise de l'état initiale
        	{
            obs[listObj[i].numObs[obs_deactive[m]]].active=0;
            obs_updated[listObj[i].numObs[obs_deactive[m]]]++;
        	}
        l=0;

		#if DEBUG
        	printf("objectif n°%hhi avec ratio=%f \n\n",i,tmp_val2);
		#endif

        //Actualisation meilleur objectif
        if (tmp_val2 > tmp_val)
            {
            tmp_val = tmp_val2;
            tmp_inx = i;
            memcpy(&path, &path_loc2, sizeof(path));
            obs[N-1].c.x = pointactuel.x;
            obs[N-1].c.y = pointactuel.y;
            obs_updated[N-1]++;
            pt_select.x = obs[N-1].c.x;
            pt_select.y = obs[N-1].c.y;
            }
        }
    if(part!=-1) //temporaire pour acceder au centre du feu
        {
        for(j=0 ; j<listObj[current_obj].nbObs ; j++)
        	{
            obs[listObj[part].numObs[j]].active=1;
            obs_updated[listObj[part].numObs[j]]++;
        	}
        part=-1;
        }
	#if DEBUG
		printListObj();
		printf("Objectif sélectionné : %i\n\n",tmp_inx);
	#endif

    return (tmp_inx);
    }

int test_tirette()
    {
    return(1);
    }

void obj_tree(iABObs_t obj) //TODO a refaire avec trajectoire programmer et fonction de selection EP
    {
    int static first=0;

    if(first==0)
        {
        printf("Debut objectif arbre\n");
        last_time=millis();
        listObj[obj].active=0;
        listObj[obj].dist=0;
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
        obs_updated[N-1]++;

        obs[obj+1].r=R_ROBOT + 5;
        obs_updated[obj+1]++;

        first=1;
        printf("Objectif : arbre n°%hhi en cour\n\n\n",obj);
        }
    else
        {

        if( (millis()-last_time)>1000)
            {
            printf("Arbre en cour\n");
            last_time=millis();
            fill_tgts_lnk();
            a_star(A(0), A(N-1), &path);
            printf("start x=%f y=%f end x=%f y=%f\n",obs[0].c.x,obs[0].c.y,obs[N-1].c.x,obs[N-1].c.y);
            send_robot(path) ;
            listObj[obj].dist=path.dist;
            }
        if ((fabs(pt_select.x-_current_pos.x)<1 && fabs(pt_select.y-_current_pos.y)<1))
            {
            mode_obj=0;
            first=0;
            pt_select.x=0;
            pt_select.y=0;
            listObj[obj].dist=0;
            }
        }
    }

void obj_bac(iABObs_t obj)
    {
    listObj[obj].active=0;
  //  obs[listObj[obj].numObs[0]].active=0;
    obs_updated[listObj[obj].numObs[0]]++;
    bac.nb_point=0;
    #if DEBUG
        printf("Objectif : bac fini\n\n\n");
    #endif
    }



int test_in_obs() //retourne le numéros de l'obstable si la position est a l'interieur de celui ci
    {        //FIXME si le robot dans plusieurs obstable
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

void obj_step(){
	int i,j;
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
        if (millis()-_start_time > END_MATCH) state = SHUT_DOWN;

        //Test si tous objectif sont fini
        temp=0;
        for(j=0 ; j<NB_OBJ ; j++)
        {
            if(listObj[j].active==0) temp++;
        }
        if(temp==NB_OBJ) state = SHUT_DOWN;

        if( (((millis()-last_time)>1000) && (mode_obj==0)) || prio==1) //calcul objectif suivant toute les seconde
        {
            prio=0;
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
            memcpy(&obs[0].c,&_current_pos, sizeof(obs[0].c));
            obs_updated[0]++;
            //FIXME
            //printf("Position actuel avant correction : x=%f et y=%f\n", _current_pos.x,_current_pos.y);
            //for(j=0 ; j<listObj[current_obj].nbObs;j++) obs[(listObj[current_obj].listIABObs[j])].active=0;
            if(((i=test_in_obs())!=0) )
            {
                project_point(_current_pos.x, _current_pos.y, obs[i].r, obs[i].c.x,obs[i].c.y, &_current_pos);;
                if(sqrt(pow(_current_pos.x-obs[0].c.x,2)+pow(_current_pos.y-obs[0].c.y,2)<2))
                {
                    memcpy(&obs[0].c,&_current_pos, sizeof(obs[0].c));
                    obs_updated[0]++;
                }
                else
                {
                    memcpy(&_current_pos,&obs[0].c, sizeof(obs[0].c));
                    obs_updated[0]++;
                }

            }
            //for(j=0 ; j<listObj[current_obj].nbObs;j++) obs[(listObj[current_obj].listIABObs[j])].active=1;


            if((millis()-last_time2)>1000)
            	{
                last_time2 = millis();
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
                //state = SHUT_DOWN; //temporaire
                break;
            case E_FEU :
                listObj[current_obj].active=0;
                listObj[current_obj].dist=0;

                for(i=0 ; i<listObj[current_obj].nbObs ; i++)
                    {
                    obs[listObj[current_obj].numObs[i]].active=0;
                    obs_updated[listObj[current_obj].numObs[i]]++;
                    }

                part=current_obj;
                break;
            default:
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

int obj_init()
	{
    if(sizeof(obs)/sizeof(*obs) != N)
    	{
        printf("N isn't correct, byebye\n");
        exit(1);
    	}

    // setting initial position
    if(COLOR==1)
        {
        obs[0].c.x=300. - 16.;
        obs[0].c.y=200. - 16.;
        }
    else
        {
        obs[0].c.x=16.;
        obs[0].c.y=200. - 16.;
        }

    obs_updated[0]++;
    _current_pos=obs[0].c;

    init_ele();

	#if DEBUG
		printListObj();
	#endif

    return 0;
	}
