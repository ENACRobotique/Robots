/*
 * mtio.h
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_MATRIXTOOLS_MT_IO_H_
#define LIB_MATRIXTOOLS_MT_IO_H_

#include <mt_mat.h>
#include <stdio.h>

void mt_v_foutput(const MT_VEC* v, FILE* f);
static inline void mt_v_output(const MT_VEC* v){
    mt_v_foutput(v, stdout);
}

void mt_m_foutput(const MT_MAT* M, FILE* f);
static inline void mt_m_output(const MT_MAT* M){
    mt_m_foutput(M, stdout);
}

#endif /* LIB_MATRIXTOOLS_MT_IO_H_ */
