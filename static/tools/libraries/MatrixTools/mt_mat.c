/*
 * mt_mat.c
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#include <stdint.h>
#include <stdlib.h>

#include "mt_mat.h"

#define MRC(m, r, c) (m)->me[(r)*(m)->cols + (c)]
#define M64(m, r, c) (int64_t)(MRC(m, r, c))

/**
 * Initializes a new matrix of size rows x cols
 * Assumes the input matrix is garbage,
 * does not frees any already allocated memory
 */
void mt_m_init(MT_MAT* m, uint8_t rows, uint8_t cols, uint8_t shift) {
    m->rows = rows;
    m->cols = cols;
    m->shift = shift;
    m->stack = 0;
    if (rows > 0 && cols > 0) {
        m->me = (int32_t*) calloc(rows * cols, sizeof(*m->me));
    }
    else {
        m->me = NULL;
    }
}

/**
 * Matrix-Vector multiplication
 * returns  0 in case of success
 *         -1 if bad input
 *         -2 if bad output
 */
int mt_mv_mlt(const MT_MAT* M, const MT_VEC* v, MT_VEC* out) {
    int i, j;
    int64_t sum;

    if (M->cols != v->elts) {
        return -1;
    }

    if (out->elts != M->rows) {
        return -2;
    }

    out->shift = v->shift;

    for (i = 0; i < M->rows; i++) {
        sum = 0;
        for (j = 0; j < M->cols; j++) {
            sum += M64(M, i, j) * (int64_t) v->ve[j];
        }
        out->ve[i] = sum >> M->shift;
    }

    return 0;
}

/**
 * Matrix-Vector multiplication + addition
 * Performs out = v1 + k*M*v2
 *   k integer must be specified with the same shift as the vectors
 * returns  0 in case of success
 *         -1 if bad input
 *         -2 if bad output
 */
int mt_mv_mltadd(const MT_VEC* v1, int32_t k, const MT_MAT* M, const MT_VEC* v2, MT_VEC* out) {
    int i, j;
    int64_t sum;

    if (M->cols != v2->elts || M->rows != v1->elts || v1->shift != v2->shift) {
        return -1;
    }

    if (out->elts != M->rows) {
        return -2;
    }

    out->shift = v1->shift;

    for (i = 0; i < M->rows; i++) {
        sum = 0;
        for (j = 0; j < M->cols; j++) {
            sum += M64(M, i, j) * (int64_t) v2->ve[j];
        }
        out->ve[i] = v1->ve[i] + (int32_t) (((int64_t) k * (sum >> M->shift)) >> v1->shift);
    }

    return 0;
}

/**
 * Matrix-Matrix multiplication
 * returns  0 in case of success
 *         -1 if bad input
 *         -2 if bad output
 */
int mt_mm_mlt(const MT_MAT* A, const MT_MAT* B, MT_MAT* OUT) {
    int i, j, k;
    int64_t sum;

    if (A->cols != B->rows || A->shift != B->shift) {
        return -1;
    }

    if (OUT->rows != A->rows || OUT->cols != B->cols) {
        return -2;
    }

    OUT->shift = A->shift;

    for (i = 0; i < OUT->rows; i++) {
        for (j = 0; j < OUT->cols; j++) {
            sum = 0;
            for (k = 0; k < A->cols; k++) {
                sum += M64(A, i, k) * M64(B, k, j);
            }
            MRC(OUT, i, j)= sum >> A->shift;
        }
    }

    return 0;
}

/**
 * Matrix inversion
 * returns  0 in case of success
 *         -1 if bad input
 *         -2 if bad output
 *         -3 if unsupported operation
 */
int mt_m_inv(const MT_MAT* M, MT_MAT* OUT) {
    int32_t det;

    if (M->rows == 1 || M->rows != M->cols) {
        return -1;
    }

    if (M->rows > 3) {
        return -3;
    }

    if (OUT->rows != M->rows || OUT->cols != M->cols) {
        return -2;
    }

    OUT->shift = M->shift;

    if (M->rows == 2) {
        det = (int32_t) ((M64(M, 0, 0) * M64(M, 1, 1) - M64(M, 0, 1) * M64(M, 1, 0)) >> M->shift);

        MRC(OUT, 0, 0)= (M64(M, 1, 1)<<M->shift)/det;
        MRC(OUT, 0, 1)= -(M64(M, 0, 1)<<M->shift)/det;
        MRC(OUT, 1, 0)= -(M64(M, 1, 0)<<M->shift)/det;
        MRC(OUT, 1, 1)= (M64(M, 0, 0)<<M->shift)/det;
    }
    else if (M->rows == 3) {
        det = (int32_t) ((M64(M, 0, 0) * ((M64(M, 1, 1) * M64(M, 2, 2) - M64(M, 2, 1) * M64(M, 1, 2)) >> M->shift) -
                        M64(M, 0, 1) * ((M64(M, 1, 0) * M64(M, 2, 2) - M64(M, 1, 2) * M64(M, 2, 0)) >> M->shift) +
                        M64(M, 0, 2) * ((M64(M, 1, 0) * M64(M, 2, 1) - M64(M, 1, 1) * M64(M, 2, 0)) >> M->shift)) >> M->shift);

        MRC(OUT, 0, 0)= (M64(M, 1, 1) * M64(M, 2, 2) - M64(M, 2, 1) * M64(M, 1, 2)) / det;
        MRC(OUT, 0, 1)= (M64(M, 0, 2) * M64(M, 2, 1) - M64(M, 0, 1) * M64(M, 2, 2)) / det;
        MRC(OUT, 0, 2)= (M64(M, 0, 1) * M64(M, 1, 2) - M64(M, 0, 2) * M64(M, 1, 1)) / det;
        MRC(OUT, 1, 0)= (M64(M, 1, 2) * M64(M, 2, 0) - M64(M, 1, 0) * M64(M, 2, 2)) / det;
        MRC(OUT, 1, 1)= (M64(M, 0, 0) * M64(M, 2, 2) - M64(M, 0, 2) * M64(M, 2, 0)) / det;
        MRC(OUT, 1, 2)= (M64(M, 1, 0) * M64(M, 0, 2) - M64(M, 0, 0) * M64(M, 1, 2)) / det;
        MRC(OUT, 2, 0)= (M64(M, 1, 0) * M64(M, 2, 1) - M64(M, 2, 0) * M64(M, 1, 1)) / det;
        MRC(OUT, 2, 1)= (M64(M, 2, 0) * M64(M, 0, 1) - M64(M, 0, 0) * M64(M, 2, 1)) / det;
        MRC(OUT, 2, 2)= (M64(M, 0, 0) * M64(M, 1, 1) - M64(M, 1, 0) * M64(M, 0, 1)) / det;
    }

    return 0;
}

void mt_m_free(MT_MAT* m) {
    if (m->me && !m->stack) {
        free(m->me);
    }
}