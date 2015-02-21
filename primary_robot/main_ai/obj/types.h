/*
 * obj_types.h
 *
 *  Created on: 17 févr. 2015
 *      Author: seb
 */

#ifndef OBJ_TYPES_H_
#define OBJ_TYPES_H_

#include "math_types.h"

typedef enum {
    E_NULL, E_CLAP, E_SPOT
} eObj_t;

typedef enum {
    WAIT_MES, ACTIVE, FREE
} eStateObj_t;

typedef struct {
        sPt_t c;
        float radiusEP;             //taille des 3 cercle d'approche
        float angleEP;                //angle d'approche entre 0 et 2 M_PI //go to the center
} sObjPt_t;

#define SPEED 1


#endif /* OBJ_TYPES_H_ */
