/*
 * ivec.h
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#ifndef MATRIXTOOLS_IVEC_H_
#define MATRIXTOOLS_IVEC_H_

#include <stdint.h>
#include <stdlib.h>
#include <alloca.h>

#define MT_VEC_SHIFT (10)

typedef struct {
    int32_t* ve;

    int elts :31;
    int8_t stack :1;
} MT_VEC;

/**
 * Allocate a vector on the stack
 * You don't need to call mt_v_free() on the objects statically initialized with this macro
 * The memory reserved for those objects will automatically be released at the end of the function where they have been initialized
 */
#define MT_V_INITS(elts) {(int32_t*)alloca((elts)*sizeof(int32_t)), (elts), 1}

void mt_v_init(MT_VEC* v, int elts);
void mt_v_free(MT_VEC* v);

#endif /* MATRIXTOOLS_IVEC_H_ */
