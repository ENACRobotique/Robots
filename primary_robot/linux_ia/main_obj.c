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
sMsg inMsg, outMsg;
int mode_obj=0;



void update_end (sNum_t radius,  sPt_t *center)
	{
    obs[N-1].c = *center;
    obs[N-1].r = radius;
    obs[N-1].active = 1;
	}

void active (sObs_t *obs) {obs->active = 1;}
void unactive (sObs_t *obs) {obs->active = 0;}



sNum_t val_obj(int num, sPath_t *path_loc) //numeros de l'objectif dans list_obj[] compris entre 0 et NB_OBJ
	{
	#if DEBUG
		printf("Debut val_obj\n");
	#endif

	sNum_t speed=1; // in m/s //TODO Faire pour trajectoire circulaire
	sNum_t time;
	sNum_t dist;
	sNum_t point;
	sNum_t ratio;
	printf("A0 x=%f & y=%f\n", obs[0].c.x, obs[0].c.y);
	printf("AN-1 x=%f & y=%f\n", obs[N-1].c.x, obs[N-1].c.y);
	fill_tgts_lnk();
	a_star(A(0), A(N-1), path_loc);
	dist=path_loc->dist;
	printf("fin a_star dist=%f\n",dist);
	if(dist==0) return (-1); //On est sur l'objectif
	time=dist/speed;
	if(time > (END_MATCH-(millis()-_start_time))) //temps restant insuffisant
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
        	printf("ratio_arbre()=%f et n=%d\n",ratio_arbre(),num);
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
    sPath_t path_loc={.dist = 0.,  .path = NULL };
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
    			printf("Next obj AN-1 x=%f & y=%f\n", obs[N-1].c.x, obs[N-1].c.y);
    			tmp_val2 = val_obj(i, &path_loc);

    			memcpy(&obs[N-1], &(((Obj_arbre*)list_obj[i].type_struct)->entrer2), sizeof(obs[N-1]));
    			printf("Next obj AN-1 x=%f & y=%f\n", obs[N-1].c.x, obs[N-1].c.y);
    			tmp_val3=val_obj(i, &path_loc);

    			if(i==0)
    			{
    				printf("y1=%f et ratio=%f\n",((Obj_arbre*)list_obj[i].type_struct)->entrer1.c.y,tmp_val2);
       				printf("y2=%f et ratio=%f\n",((Obj_arbre*)list_obj[i].type_struct)->entrer2.c.y,tmp_val3);
    			}
    			if(tmp_val3>tmp_val2) tmp_val2=tmp_val3;
    			else
    				{
    				memcpy(&obs[N-1], &(((Obj_arbre*)list_obj[i].type_struct)->entrer1), sizeof(obs[N-1]));
    				val_obj(i, &path_loc);
    				printf("non\n");
    				}

    			obs[i+1].active=1;
    			break;
        	case E_BAC :
        		obs[i+1+5].active=0;
        		obs[i+2+5].active=0;
        		obs[i+3+5].active=0;
        		memcpy(&obs[N-1], &(((Obj_bac*)list_obj[i].type_struct)->entrer), sizeof(obs[N-1]));
        		tmp_val2 =val_obj(i, &path_loc);
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
				memcpy(&path, &path_loc, sizeof(path));
			    pt_select.x=obs[N-1].c.x;
			    pt_select.y=obs[N-1].c.y;
				}
			if(tmp_inx != -1)
				{
				obs[tmp_inx+1].r=R_ROBOT + 5;
				}
    	}
 //   printf("pos select y=%f\n",path.
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
				printf("received position (%.1fcm,%.1fcm,%.2f°)\n", inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta*180./M_PI);

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

void obj_tree(iABObs_t obj)
	{
	list_obj[obj].active=0;
	bac.nb_point=bac.nb_point+3;
	mode_obj=1;

	if(pt_select.x==(((Obj_arbre*)list_obj[obj].type_struct)->entrer1).c.x && pt_select.y==(((Obj_arbre*)list_obj[obj].type_struct)->entrer1).c.y)
		{
		pt_select=((Obj_arbre*)list_obj[obj].type_struct)->entrer2.c;
		printf("Sortie 2 selectionné\n");
		}
	else
		{
		pt_select=((Obj_arbre*)list_obj[obj].type_struct)->entrer1.c;
		printf("Sortie 1 selectionné\n");
		}

	obs[N-1].c.x=pt_select.x;
	obs[N-1].c.y=pt_select.y;

	printf("Objectif : arbre n°%hhi en cour\n\n\n",obj);
	}

void obj_bac(iABObs_t obj)
	{
	list_obj[obj].active=0;
	bac.nb_point=0;
	#if DEBUG
		printf("Objectif : bac fini\n\n\n");
	#endif
	}
void obj_tree2(iABObs_t obj)
	{
	list_obj[obj].active=0;
	#if DEBUG
		printf("Objectif : arbre fondn°%hhi fini\n\n\n",obj);
	#endif
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

int test_in_obs() //retourne le numéros de l'obstable si la position est a l'interieur de celui ci
	{		//FIXME si le robot dans plusieur obstable
	int i;
	for(i=1; i<N-2;i++)
		{
		if( sqrt(pow(obs[i].c.x-_current_pos.x,2)+pow(obs[i].c.y-_current_pos.y,2)) < obs[i].r) return i;
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

                //TEST projection
//                project_point(-16, -45, 10, 5, 0, &point);
//                printf("x=%f & y=%f\n", point.x, point.y);


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

					  if( ((millis()-last_time)>1000) && (mode_obj==0))
					  	  {
						  printf("A0 x=%f & y=%f\n", obs[0].c.x, obs[0].c.y);
						  current_obj = next_obj();

						  //  getchar();
						  if ( current_obj == -1) continue; //aucun objectif actif atteignable
						  current_obs = (list_obj[current_obj]).num_obs ;//Conversion objectif -> obstable

						  send_robot(path) ;// Envoie au bas niveau la trajectoire courante
						  last_time=millis();
					  	  }
					  if(get_position(&_current_pos))
					  	  {
						  printf("Position actuel avant correction : x=%f et y=%f\n", obs[0].c.x,obs[0].c.y);

						  if((i=test_in_obs())!=0) project_point(_current_pos.x, _current_pos.y, obs[i].r, obs[i].c.x,obs[i].c.y, &_current_pos);;
						  //					  _current_pos.x=_current_pos.x+0.1;
						  //					  _current_pos.y=_current_pos.y+0.1;
						  memcpy(&obs[0].c,&_current_pos, sizeof(obs[0].c));
						  printf("Position actuel : x=%f et y=%f\n", _current_pos.x,_current_pos.y);
						  printf("select : x=%f et y=%f avec fabsx=%f et fabsy=%f\n", pt_select.x,pt_select.y, fabs(pt_select.x-_current_pos.x),fabs(pt_select.y-_current_pos.y));

						  if( (millis()-last_time)>1000 &&mode_obj==1)
							  {
							  fill_tgts_lnk();
							  a_star(A(0), A(N-1), &path);
							  printf("Objectif atteint : dist local=%l",path.dist);
							  send_robot(path) ;
							  }

						  if (fabs(pt_select.x-_current_pos.x)<RESO_POS && fabs(pt_select.y-_current_pos.y)<RESO_POS)   //objectif atteint
							  {
							  if(mode_obj==1)mode_obj=0;
							  else
							  {
								  printf("Un objectif a été atteint\n");
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
	int i, ret;

	//Position de départ
	obs[0].c.x=1;
	obs[0].c.y=199;

	//initialisation de la communication
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
    	}

#if SEB
	obs[0].active = 1;
    obs[2].active = 1;
    obs[3].active = 1;

    //Changement de la destination
    obs[N-1].c.x = 80;
    obs[N-1].c.y = 180;
    obs[N-1].active = 1;

#else
    printf("Debut initialisation element du jeu\n");
    //Activation des éléments du jeu
	int j;
	for(i=0; i<N;i++ )
		{
		obs[i].active =1;
		}
	//Initialisation des arbres
	printf("Initialisation du bac\n");
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
		printf("Initialisation du bac\n");
		list_obj[5].type_struct = &bac;
		((Obj_bac*)list_obj[i].type_struct)->nb_point=0;
		((Obj_bac*)list_obj[i].type_struct)->entrer=obs_PA[8];

	//TODO Initialisation des autres éléments
	//TODO activation bac jaune ou rouge

#endif



	//Initialisation de la position courante;
	_current_pos=obs[0].c;


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

#if DEBUG
	printf("Fin d'initialisation des éléments de jeu\n");
#endif

    _start_time = millis();
    state_machine();

    return 0;
}

