/*
 * mt_io.h
 *
 *  Created on: 15 févr. 2015
 *      Author: ludo6431
 */

#ifndef MT_IO_H_
#define MT_IO_H_

#include <stdio.h>

#include "mt_vec.h"
#include "mt_mat.h"

void mt_v_foutput(const MT_VEC* v, FILE* f);
static inline void mt_v_output(const MT_VEC* v){
    mt_v_foutput(v, stdout);
}

void mt_m_foutput(const MT_MAT* M, FILE* f);
static inline void mt_m_output(const MT_MAT* M){
    mt_m_foutput(M, stdout);
}

#endif /* MT_IO_H_ */
