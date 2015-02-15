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

#define MT_VEC_SHIFT (10)

typedef struct {
    int32_t* ve;

    int elts;
} MT_VEC;

void mt_v_init(MT_VEC* v, int elts);
void mt_v_free(MT_VEC* v);

#endif /* MATRIXTOOLS_IVEC_H_ */
