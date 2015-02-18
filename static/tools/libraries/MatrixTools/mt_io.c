/*
 * mt_io.c
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#include "mt_io.h"

#define MRC(m, r, c) (m)->me[(r)*(m)->cols + (c)]

void mt_v_foutput(const MT_VEC* v, FILE* f) {
    int i;

    fprintf(f, "size %i:\n", v->elts);

    if (v->elts < 1) {
        return;
    }

    for (i = 0; i < v->elts; i++) {
        fprintf(f, "  % 10.6f\n", (double) v->ve[i] / (double) (1 << v->shift));
    }
}

void mt_m_foutput(const MT_MAT* M, FILE* f) {
    int i, j;

    fprintf(f, "size %ix%i:\n", M->rows, M->cols);

    if (M->rows < 1 || M->cols < 1) {
        return;
    }

    for (i = 0; i < M->rows; i++) {
        fprintf(f, "  % 10.6f", (double) MRC(M, i, 0) / (double) (1 << M->shift));
        for (j = 1; j < M->cols; j++) {
            fprintf(f, ", % 10.6f", (double) MRC(M, i, j) / (double) (1 << M->shift));
        }
        fprintf(f, "\n");
    }
}
