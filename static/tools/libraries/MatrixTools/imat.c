/*
 * imatrix.c
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#include <stdint.h>

#include "imat.h"

#define MRC(m, r, c) (m)->me[(r)*(m)->cols + (c)]

MT_MAT* mt_m_get(int rows, int cols) {
    MT_MAT* m = (MT_MAT*) malloc(sizeof(*m));
    if (m) {
        m->rows = rows;
        m->cols = cols;
        m->me = (int*) calloc(rows * cols, sizeof(*m->me));
    }
    return m;
}

MT_VEC* mt_mv_mlt(const MT_MAT* M, const MT_VEC* v, MT_VEC* out) {
    int i, j;
    int64_t sum;

    if (out) {
        if (out->elts != M->rows) {
            return MT_VNULL;
        }
    }
    else {
        out = mt_v_get(M->rows);
    }

    for (i = 0; i < M->rows; i++) {
        sum = 0;
        for (j = 0; j < M->cols; j++) {
            sum += MRC(M, i, j)* v->ve[j];
        }
        out->ve[i] = sum >> MT_MAT_SHIFT;
    }

    return out;
}

MT_MAT* mt_mm_mlt(const MT_MAT* A, const MT_MAT* B, MT_MAT* OUT) {
    int i, j, k;
    int64_t sum;

    if (A->cols != B->rows) {
        return MT_MNULL;
    }

    if (OUT) {
        if (OUT->rows != A->rows || OUT->cols != B->cols) {
            return MT_MNULL;
        }
    }
    else {
        OUT = mt_m_get(A->rows, B->cols);
    }

    for (i = 0; i < OUT->rows; i++) {
        for (j = 0; j < OUT->cols; j++) {
            sum = 0;
            for (k = 0; k < A->cols; k++) {
                sum += MRC(A, i, k)* MRC(B, k, j);
            }
            MRC(OUT, i, j)= sum >> MT_MAT_SHIFT;
        }
    }

    return OUT;
}

MT_MAT* mt_m_inv(const MT_MAT* M, MT_MAT* OUT) {
    return MT_MNULL; // TODO
}

MT_MAT* mt_m_free(MT_MAT* m) {
    if (m) {
        if (m->me) {
            free(m->me);
        }
        free(m);
    }

    return MT_MNULL;
}
