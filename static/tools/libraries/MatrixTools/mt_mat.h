/*
 * mt_mat.h
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#ifndef MT_MAT_H_
#define MT_MAT_H_

#include <stdint.h>
#include <alloca.h>

#include "mt_vec.h"

typedef struct {
    int32_t* me;

    uint8_t rows;
    uint8_t cols;
    uint8_t shift;
    uint8_t stack;
} MT_MAT;

/**
 * Allocate a matrix on the stack
 * You don't need to call mt_m_free() on the objects statically initialized with this macro
 * The memory reserved for those objects will automatically be released at the end of the function where they have been initialized
 */
#define MT_M_INITS(rows, cols, shift) {(int32_t*)alloca((rows)*(cols)*sizeof(int32_t)), (rows), (cols), (shift), 1}

#define MT_M_AT(m, r, c) (m)->me[(r)*(m)->cols + (c)]

void mt_m_init      (MT_MAT* m, uint8_t rows, uint8_t cols, uint8_t shift);
int  mt_mm_add      (const MT_MAT* A, const MT_MAT* B, MT_MAT* OUT);
int  mt_m_shift     (const MT_MAT* A, const uint8_t new_shift, MT_MAT* OUT);
int  mt_m_mltshift  (const MT_MAT* A, const uint8_t new_shift, const int8_t data_shift, MT_MAT* OUT);
int  mt_mv_mlt      (const MT_MAT* M, const MT_VEC* v, MT_VEC* out);
int  mt_mv_mltadd   (const MT_VEC* v1, int32_t k, const MT_MAT* M, const MT_VEC* v2, MT_VEC* out);
int  mt_mm_mlt      (const MT_MAT* A, const MT_MAT* B, MT_MAT* OUT);
int  mt_mtdm_mlt    (const MT_MAT* A, const MT_VEC* d, MT_MAT* OUT);
int  mt_m_inv       (const MT_MAT* M, MT_MAT* OUT);
void mt_m_free      (MT_MAT* m);

#endif /* MT_MAT_H_ */
