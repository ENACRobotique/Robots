/*
 * ivec.c
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#include <mt_vec.h>

void mt_v_init(MT_VEC* v, int elts) {
    v->elts = elts;
    if (elts > 0) {
        v->ve = (int32_t*) calloc(elts, sizeof(*v->ve));
    }
    else {
        v->ve = NULL;
    }
}

void mt_v_free(MT_VEC* v) {
    if (v->ve) {
        free(v->ve);
    }
}
