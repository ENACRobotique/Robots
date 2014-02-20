#include <time.h>
#include <unistd.h>

#include "a_star.h"
#include "math_types.h"
#include "math_ops.h"
#include "tools.h"


#define SEB 1
#define Nb_obs_start 41

typedef enum{E_VERRE, E_CADAL, E_BOUGIE , E_MAISON } eObj_t;

typedef struct {
    eObj_t type;
    sNum_t test;
    iABObs_t obs;
    sPt_t *position;
} sObj_t;

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)>(b)?(b):(a))
#define CLAMP(m, v, M) MAX(m, MIN(v, M))

// constantes qui vont bien TODO TODO TODO !!!!!!
sNum_t t_lastg_l = 0.;   // time_lastglass left
sNum_t t_lastg_r = 1000.;
sNum_t av_occ_t = 2000.;    // average occupation time
int gnb_l,gnb_r; // glass number right/left
sNum_t pts_per_msec = 128./90000.;

typedef struct {

    int type;


} sMess_t;

//Initialisation des éléments de jeu
	sPath_t path= {.dist = 0.,  .path = NULL };

	//structure arbre
	typedef struct{	//TODO définir l'entré de l'arbre en fonction des fruits
		uint8_t fruit_1; //définition : 0=violet non récolté, 1=récolté, 2=noir
		uint8_t fruit_2;
		uint8_t fruit_3;
		uint8_t fruit_4;
		uint8_t fruit_5;
		uint8_t fruit_6;
		//TODO orientation du robot sur cible
		uint8_t nb_point;
		uint8_t dist;
		uint8_t prio;
		} arbre;

	sObs_t arbre_1a ={{10., 90.}, 0, 0, 1}; //1ere entré sur l'objectif arbre 1
	sObs_t arbre_1b ={{10., 50.}, 0, 0, 1};
	//TODO idem pour les 3 autres arbres

	//Obstacle
		sObs_t obs2[] = {
			// foyer
				{{0., 0.}, R_ROBOT+25, 1, 1},
				{{300., 0.}, R_ROBOT+25, 1, 1},
				{{150., 95.}, R_ROBOT+30, 1, 1},
			// torche mobile
				{{90., 90.}, R_ROBOT+16., 1, 1},
				{{210., 90.}, R_ROBOT+16, 1, 1},
			//torche fixe
				{{0., 120.}, R_ROBOT+5, 1, 1},
				{{130., 0.}, R_ROBOT+5, 1, 1},
				{{170., 0.}, R_ROBOT+5, 1, 1},
				{{300., 120.}, R_ROBOT+5, 1, 1},
			};


	sObs_t bougie_a = {{218., 179.}, R_ROBOT, 0, 0};
	sObs_t bougie_b = {{83., 180.}, R_ROBOT, 0, 0}; //FIXME

// TODO initialise obj_pts 0-1 bougies / 2-9 cadeaux / 10-21 verres / 22 retour maison TODO
sObj_t obj_pts[]={ //mettre a jour liste des objectifs
    {E_VERRE, 1., A(2), NULL},
    {E_VERRE, 1., A(3), NULL},
    {E_BOUGIE, 1., A(N-1), &bougie_a.c}
};

#define NB_OBJ ((int)sizeof(obj_pts)/(int)sizeof(*obj_pts))

unsigned long _start_time;
unsigned long millis()
	{
    unsigned long res = (time(NULL) - _start_time)*1000;
    //printf("millis()=%lu\n", res);
    return res;
	}




void update_end (sNum_t radius,  sPt_t *center) {

    obs[N-1].c = *center;
    obs[N-1].r = radius;
    obs[N-1].active = 1;

}

void active (sObs_t *obs) {obs->active = 1;}
void unactive (sObs_t *obs) {obs->active = 0;}

int gat_ind;

sNum_t val_obj( sObj_t *objc , sNum_t av_speed)
	{

    unsigned long current_t = millis();
    sNum_t margin = 500.; // TODO def margin
    sNum_t g_bougie, g_verre, d_home;
    g_bougie = 4.;
    //sNum_t g_cadal = 4.;

//FIXME valeurs en dur! rayons
   // sNum_t r_cadal = 4.;
    sNum_t r_bougie = R_ROBOT;
    sNum_t r_verre = 15.; // h TODO




    switch( objc -> type) {

        case E_BOUGIE :

            update_end(r_bougie, objc -> position);

            fill_tgts_lnk();

            a_star(0, B(N-1), &path);
            printf("val_obj dist(curr,bougie%u%c)=%.2f\n", O(objc->obs), DIR(objc->obs)?'b':'a', path.dist);

            return(path.dist?g_bougie * av_speed / path.dist:0);
            break;

/* cadeaux

        case E_CADAL :

            update_end(r_cadal, objc -> position);

            //  écrire dans le tab des obstacles, dans la case 2N-1, la positon du pt cadal
            a_star(0, 2*N-1 , &path);
        return(path.dist?g_cadal * av_speed / path.dist:0);
        break;
*/
        case E_VERRE :
            update_end(r_verre, &obs[O(objc -> obs)].c);
            unactive(& obs[O(objc -> obs)]);

            fill_tgts_lnk();

            if (objc -> obs & 1) {

                sNum_t t_wait = - (sNum_t)current_t + t_lastg_l + av_occ_t; // time to wait before glass stack available
                if (t_wait < 0.) g_verre = 4.*objc->test*(gnb_l+1.);
                else g_verre = (4.*(gnb_l+1.) - t_wait*pts_per_msec)*objc->test;
           }
           else{
                sNum_t t_wait = - (sNum_t)current_t + t_lastg_r + av_occ_t;
                if (t_wait < 0.) g_verre = 4.*(gnb_r+1.)*objc->test;
                else {
                g_verre = (4.*(gnb_r+1.) - t_wait*pts_per_msec)*objc->test;
            }
            }
//            printf("g_verre=%.2f\n", g_verre);
            a_star(0, A(N-1) + DIR(objc -> obs), &path);
            printf("val_obj dist(curr,verre%u%c)=%.2f\n", O(objc->obs), DIR(objc->obs)?'b':'a', path.dist);

            active(& obs[O(objc -> obs)]);

            return(path.dist?g_verre * av_speed / path.dist:0);

            break;

        case E_MAISON :

            update_end(0., objc -> position);

            fill_tgts_lnk();

            a_star(0, A(N-1), &path);
            d_home =  path.dist;
            if (av_speed*(90000. - current_t + margin) > d_home) return(0.);
            else return (9001.);

        default:
            break;
    }

    return 0.;
}

iABObs_t next_obj (int* test_bougie) //retourne l'objectif suivant
	{


    int i;

    sNum_t tmp_val = 0.;
    sNum_t tmp_val2;
    int tmp_inx=-1;

    for(i = 0 ; i < NB_OBJ ; i++)
    	{
        if (obj_pts[i].test)
        tmp_val2 = val_obj( &obj_pts[i], 0.5); //FIXME speed codée en dur !
        printf("next_obj g[%u%c]=%.4f\n", O(obj_pts[i].obs), DIR(obj_pts[i].obs)?'b':'a', tmp_val2);

            if (tmp_val2 > tmp_val) {
                tmp_val = tmp_val2;
                tmp_inx = i;
            }
    }
    if (obj_pts[tmp_inx].type == E_BOUGIE) *test_bougie = 1;

    else *test_bougie = 0;

    return ((iABObs_t)tmp_inx);
	}

/*

typedef struct {
    sPt_t p1;
    sPt_t p2;
    sObs_t obs;

    unsigned short sid;
} sTrajEl_t;

typedef struct {
    sNum_t dist;
    unsigned short tid;

    unsigned int path_len;
    sTrajEl_t *path;
} sPath_t;

typedef struct {
    sPt_t c;    // center of obstacle
    sNum_t r;   // radius

    uint8_t moved:4;
    uint8_t active:4;
} sObs_t;
*/



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

typedef enum {ATTENTE , JEU , SHUT_DOWN} estate_t;

int distance_bougie (sObj_t ** bougie_ind){
    sNum_t d1, d2;
    sqdistPt2Pt(obj_pts[1].position,&(obs[0].c), &d1); // FIXME FIXME bougie dans obs ou obj ou les 2? les 2!
    sqdistPt2Pt(obj_pts[2].position,&(obs[0].c), &d2);
    if (d1 > 1 && d2 > 1) return 0;
    else {
        if (d1 < 1) *bougie_ind = &obj_pts[2];
        else *bougie_ind = &obj_pts[1];
        }
    return 1 ;
}
void get_position() { //TODO

return;
}

void state_machine()
	{
    gat_ind = 1;
    estate_t state = ATTENTE;
    iABObs_t current_obj;

    sPath_t current_path, next_path;
    current_path.tid = 1;
    next_path.tid = 1;

   // int test_bougie = 0;

    sPath_t *_current_path = &current_path;
    sPath_t *_next_path = &next_path;

    //sObj_t *bougie_ind = NULL;
#if 0
		sMsg msg;
#endif

    while(1){

        switch (state) {

            case ATTENTE :
                printf("Attente. time = %ld\n", millis()); // FIXME debug
                if ( test_tirette() )
                	{
                	state = JEU;
                	_start_time = time(NULL);
                	}
        		break;

            case JEU :
                if (millis() > 90000) state = SHUT_DOWN;
#if 0
                sb_receive(&msg);
                else {
                    if (msg.header.type == E_POS) {

                    else
#endif

                    {
                          // TEST prog
                              sNum_t xpos, ypos; //FIXME debug
                              printf("Jeu. time = %ld\n", millis()); //FIXME debug
                              printf("position?\n");
                              fflush(stdout);
                              scanf("%f %f", &xpos, &ypos);
                              obs[0].c.x = xpos;
                              obs[0].c.y = ypos;
                              printf("cx %f cy %f \n",obs[0].c.x ,obs[0].c.y);
                           //END test

                              fill_tgts_lnk();

                              current_obj = next_obj(&test_bougie); //current_pos plutot ou rien
                              if ( current_obj == -1) continue; //aucun objectif actif atteignable

                                  switch (test_bougie){

                                  case 0:

                                        printf("state_m test bougie case 0\n");


                                        a_star(A(0), current_obj, _next_path);

                                        if (!(same_traj(_next_path,_current_path)))
                                        {
                                        current_path.tid++;
                                        next_path.tid = current_path.tid;
                                        _current_path = _next_path;
                                        }


                                //transfert_traj(current_path);//TODO





                                  case 1: //cas bougies

                                    printf("state_m test bougie case 1\n");

                                    if (distance_bougie (&bougie_ind)){  // TODO TODO TODO
                                    printf("switch 2.0 bougie\n");

                                     a_star(A(0),bougie_ind->obs,_current_path);


                                    }
                                    else
                                        a_star(A(0), current_obj, _current_path);
                     }

               }
            break;

            case SHUT_DOWN:
            printf ("SHUT_DOWN\n");
                //TODO arrêt total
            break;


        }
    }


    return;
}


int main()
	{
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
	int i;
	for(i=0; i<Nb_obs_start;i++ )
		{
		obs[i].active =1;
		}
#endif


    _start_time = time(NULL);
    state_machine();



 //test next_obj

/*    _start_time = time(NULL);

    obs[0].active = 1;
    obs[2].active = 1;
    obs[3].active = 1;

    obs[N-1].c.x = 80;
    obs[N-1].c.y = 180;
    obs[N-1].active = 1;

    // fill
    fill_tgts_lnk();
    int j = 10;
    while(j) {
        int res = next_obj (A(0));
        printf("res=%i\n", res);
        j--;
        usleep(500000);
    }
*/
    return 0;
}

