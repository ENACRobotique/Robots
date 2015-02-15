/*
 * ivec.c
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#include "ivec.h"

MT_VEC* mt_v_get(int elts) {
    MT_VEC* v = (MT_VEC*) malloc(sizeof(*v));
    if (v) {
        v->elts = elts;
        v->ve = (int*) calloc(elts, sizeof(*v->ve));
    }
    return v;
}

MT_VEC* mt_v_free(MT_VEC* v) {
    if (v) {
        if (v->ve) {
            free(v->ve);
        }
        free(v);
    }

    return MT_VNULL;
}
