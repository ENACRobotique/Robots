/*
 * imatrix.h
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#ifndef MATRIXTOOLS_IMAT_H_
#define MATRIXTOOLS_IMAT_H_

#include <stdlib.h>
#include <stdint.h>

#include "ivec.h"

#define MT_MAT_SHIFT (10)

typedef struct {
    int32_t* me;

    int rows;
    int cols;
} MT_MAT;

void mt_m_init(MT_MAT* m, int rows, int cols);
int  mt_mv_mlt(const MT_MAT* M, const MT_VEC* v, MT_VEC* out);
int  mt_mm_mlt(const MT_MAT* A, const MT_MAT* B, MT_MAT* OUT);
int  mt_m_inv(const MT_MAT* M, MT_MAT* OUT);
void mt_m_free(MT_MAT* m);

#endif /* MATRIXTOOLS_IMAT_H_ */
