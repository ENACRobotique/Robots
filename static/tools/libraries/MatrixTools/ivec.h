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

MT_VEC* mt_v_get(int elts);
MT_VEC* mt_v_free(MT_VEC* v);
#define MT_V_FREE(v) do { (v) = mt_v_free((MT_VEC*)(v)); } while(0)

#define MT_VNULL ((MT_VEC*)NULL)

#endif /* MATRIXTOOLS_IVEC_H_ */
