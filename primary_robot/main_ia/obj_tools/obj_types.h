#ifndef _TYPE_IA_H
#define _TYPE_IA_H

#include<time.h>

#include "tools.h"
#include "a_star.h"

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../../global_errors.h"
#include "node_cfg.h"


#define SEB 0
#define COLOR 0 //0=red and 1=yellow
#define DEBUG 1
#define Nb_obs_start 41
#define RESO_POS 2
#define NB_OBJ 15
#define END_MATCH 180000 //in ms
#define CONST_AJUST 0.1 //TODO trouver le pb a star ou fill tangente

typedef enum {ATTENTE , JEU , SHUT_DOWN} estate_t;
typedef enum {E_FEU, E_TORCHE_MOBILE, E_ARBRE, E_ARBRE_FOND , E_BAC, E_FOYER, E_TORCHE_FIXE} eObj_t;

typedef struct{
    sObs_t a;
    sObs_t b;
    sObs_t c;
    sObs_t d;
    } listObs;


//Struture d'un objetif
    typedef struct {
        eObj_t  type;            //type d'objectif
        uint8_t numObj;            //numéros de l'objectif en fonction du type
        uint8_t nbObs;            //nombre d'obstacle associé  cet objectif
        uint8_t listIABObs[5];  //numeros des obstacle associer à l'objectif
        sNum_t dist;            //distance robot-objectif(EntryPoint le plus proche)
        uint8_t active;         //0=objectif fini 1=objectif non fini
        uint8_t nbEP;            //nombre de point d'accès de l'objectif
        sObs_t  entryPoint[4];    //liste des point d'accès pour atteindre l'objectif
        void   *typeStruct;        //structure particuliere associer au type d'objectif
    } sObj_t;

//Structure arbre
typedef struct{        //TODO orientation du robot sur cible
    uint8_t eFruit[6]; //définition : 0=violet non récolté, 1=récolté, 2=noir
    uint8_t nb_point;  //nombre de point potentiel a vider
    } Obj_arbre;

//Strucutre feu
typedef struct{
    uint8_t pos;     //postition du feu TODO a définir
    uint8_t nb_point;  //nombre de point potentiel a vider
    } Obj_feu;

//Struture bac
typedef struct{
    uint8_t nb_point;
    }Obj_bac;


extern Obj_arbre arbre[4];
extern Obj_bac bac;
extern uint8_t obs_updated[N];

extern sObj_t listObj[]; //Liste des objectifs
extern sObs_t listEP[]; //Liste de tous les points d'acces des objectifs

extern sPt_t _current_pos;
extern sPath_t path;
extern long _start_time;
extern sNum_t ratio_arbre(void);
extern sNum_t ratio_bac(void);
extern long last_time;
extern long last_time2;

extern void init_ele(void);
extern void send_robot(sPath_t path);
extern int get_position( sPt_t *pos);
extern float sign(float x);
extern void project_point(float xp, float yp, float rc, float xc, float yc, sPt_t *point);
extern int init_mes(void);
extern void printListObj(void);

#endif



