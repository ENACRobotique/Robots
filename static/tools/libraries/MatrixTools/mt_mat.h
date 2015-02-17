/*
 * imatrix.h
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#ifndef MATRIXTOOLS_IMAT_H_
#define MATRIXTOOLS_IMAT_H_

#include <mt_vec.h>
#include <stdlib.h>
#include <stdint.h>
#include <alloca.h>

#define MT_MAT_SHIFT (16)

typedef struct {
    int32_t* me;

    int rows;
    int cols :31;
    int8_t stack :1;
} MT_MAT;

/**
 * Allocate a matrix on the stack
 * You don't need to call mt_m_free() on the objects statically initialized with this macro
 * The memory reserved for those objects will automatically be released at the end of the function where they have been initialized
 */
#define MT_M_INITS(rows, cols) {(int32_t*)alloca((rows)*(cols)*sizeof(int32_t)), (rows), (cols), 1}

#define MT_M_AT(m, r, c) (m)->me[(r)*(m)->cols + (c)]

void mt_m_init(MT_MAT* m, int rows, int cols);
int  mt_mv_mlt(const MT_MAT* M, const MT_VEC* v, MT_VEC* out);
int  mt_mv_mltadd(const MT_VEC* v1, int32_t k, const MT_MAT* M, const MT_VEC* v2, MT_VEC* out);
int  mt_mm_mlt(const MT_MAT* A, const MT_MAT* B, MT_MAT* OUT);
int  mt_m_inv(const MT_MAT* M, MT_MAT* OUT);
void mt_m_free(MT_MAT* m);

#endif /* MATRIXTOOLS_IMAT_H_ */
