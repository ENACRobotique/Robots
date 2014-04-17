#ifndef _TYPE_IA_H
#define _TYPE_IA_H

#include<time.h>

#include "tools.h"
#include "a_star.h"
#include "math_ops.h"

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../../global_errors.h"
#include "node_cfg.h"


#define SEB 0
#define COLOR 0 //0=red and 1=yellow
#define DEBUG 1
#define RESO_POS 2
#define NB_OBJ 16
#define END_MATCH 180000 //in ms
#define MAX_OBS_BY_OBJ 3
#define MAX_EP_BY_OBJ 3
#define START_FEU 28
#define START_ARBRE 1
#define START_TORCHE_FIXE 20
#define FIRE_RADIUS_EP 15


typedef enum {ATTENTE , JEU , SHUT_DOWN} estate_t;
typedef enum {E_FEU, E_TORCHE_MOBILE, E_ARBRE, E_ARBRE_FOND , E_BAC, E_FOYER, E_TORCHE_FIXE} eObj_t;

typedef struct
	{
	sPt_t c;
    float radiusEP;				//taille des 3 cercle d'approche
    int angleEP;				//angle d'approche entre 0 et 360
	} sObjPoint_t;


//Struture d'un objetif
typedef struct
	{
	eObj_t  type;           				//type d'objectif
	uint8_t numObj;         				//numéros de l'objectif en fonction du type
	uint8_t nbObs;          				//nombre d'obstacle associé  cet objectif
	uint8_t numObs[MAX_OBS_BY_OBJ];  	    //numeros des obstacles associer à l'objectif
	sNum_t dist;            				//distance robot-objectif(EntryPoint le plus proche)
	uint8_t active;        	 				//0=objectif fini 1=objectif non fini
	uint8_t nbEP;          	 				//nombre de point d'accès de l'objectif
	sObjPoint_t  entryPoint[MAX_EP_BY_OBJ]; //liste des point d'accès pour atteindre l'objectif
	void   *typeStruct;         			//structure particuliere associer au type d'objectif
    } sObj_t;


//Structure arbre
typedef struct          //TODO pointeur vers trajectoire programmer //TODO faire fonction calcul point d'acces optimal et calcul de la trajectoire
	{
	uint8_t eFruit[6];  //définition : 0=violet non récolté, 1=récolté, 2=noir : first top clock
    uint8_t nb_point;   //nombre de point potentiel a vider
    int x;				//position of the tree axis x
    int y;				//position of the tree axis y
    sPt_t EntryPoint1;
    sPt_t EntryPoint2;
    uint8_t rot;		//0 no rotation else rotation
    } Obj_arbre;

//Strucutre feu
typedef struct
	{
	sPt_t c; 			//center of the fire
	uint8_t pos;       	// 1 = flat red, 2 = flat yellow, 3 = vertical, 4 = in fixed torch 5=other
	sNum_t angle; 		// vertical or torch fixed in [0, 360°[ convention R|Y ->0°, flat in [0, 120°[
    uint8_t nb_point;  	//nombre de point
    } Obj_feu;

//Struture bac
typedef struct
	{
    uint8_t nb_point;
    }Obj_bac;


extern Obj_arbre arbre[4];
extern Obj_bac bac;
extern Obj_feu feu[16];

extern uint8_t obs_updated[N];

extern sObj_t listObj[];

extern sPt_t _current_pos;
extern sPath_t path;
extern long _start_time;
extern long last_time;
extern long last_time2;

extern sNum_t ratio_arbre(void);
extern sNum_t ratio_bac(void);

extern void init_ele(void);
extern void send_robot(sPath_t path);
extern int get_position( sPt_t *pos);
extern float sign(float x);
extern void project_point(float xp, float yp, float rc, float xc, float yc, sPt_t *point);

extern void printListObj(void);
extern void printObsActive(void);

extern void TransElTraj(sTrajEl_t *traj, int x, int y);
extern void SymElTraj(sTrajEl_t *traj, int x, int y);
extern void Rot90Traj(sTrajEl_t *traj);
extern sNum_t arc_len2(sPt_t *p2_1, sPt_t *oc, sNum_t or, sPt_t *p2_3);
extern float sign(float x);

#endif



