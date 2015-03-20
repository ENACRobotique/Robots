/*
 * mt_vec.c
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#include <stdlib.h>

#include "mt_vec.h"

void mt_v_init(MT_VEC* v, uint16_t elts, uint8_t shift) {
    v->elts = elts;
    v->shift = shift;
    v->stack = 0;
    if (elts > 0) {
        v->ve = (int32_t*) calloc(elts, sizeof(*v->ve));
    }
    else {
        v->ve = NULL;
    }
}

void mt_v_free(MT_VEC* v) {
    if (v->ve && !v->stack) {
        free(v->ve);
    }
}
