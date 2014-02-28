#ifndef _TYPE_IA_H
#define _TYPE_IA_H

#include<time.h>

#include "tools.h"
#include "a_star.h"

#define SEB 0
#define DEBUG 1
#define Nb_obs_start 41
#define RESO_POS 2
#define NB_OBJ 6
#define END_MATCH 90000 //in ms
#define COLOR 0 //0=red and 1=yellow
#define CONST_AJUST 0.1 //TODO trouver le pb a star ou fill tangente

typedef enum {ATTENTE , JEU , SHUT_DOWN} estate_t;
typedef enum {E_FEU, E_TORCHE_MOBILE, E_ARBRE, E_ARBRE_FOND , E_BAC, E_FOYER, E_TORCHE_FIXE} eObj_t;

//Struture d'un objetif
typedef struct {
    eObj_t type;		//type d'objectif
    iABObs_t num_obj; 	//numeros de l'objectif
    iABObs_t num_obs;	//numeros de l'obstacle
    uint8_t nb_PA;
    uint8_t active; 		//0=objectif fini 1=objectif non fini
    void *type_struct;	//struct particuliere associer au type d'objectif
} sObj_t;

//structure arbre
typedef struct{		//TODO orientation du robot sur cible
	uint8_t eFruit[6]; //définition : 0=violet non récolté, 1=récolté, 2=noir
	uint8_t nb_point; //nombre de point potentiel a vider
	uint8_t dist;
	sObs_t entrer1;
	sObs_t entrer2;
	sObs_t entrer1second;
	sObs_t entrer2second;
	} Obj_arbre;

typedef struct{
	uint8_t nb_point;
	sObs_t entrer;
	}Obj_bac;

extern Obj_arbre arbre[4];
extern Obj_bac bac;

extern sObj_t list_obj[]; //Liste des objectifs
extern sObs_t obs_PA[]; //Liste de tous les points d'acces des objectifs

extern sObs_t _current_pos;
extern sPath_t path;
extern unsigned long _start_time;
extern sNum_t ratio_arbre(void);
extern sNum_t ratio_bac(void);
extern unsigned long millis();

#endif



