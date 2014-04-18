/*
 * obj_path_tools.c
 *
 *  Created on: 17 avr. 2014
 *      Author: seb
 */

#include "obj_path_tools.h"


sNum_t seg_len(sPt_t *p1, sPt_t *p2){
	return sqrt(fabs(p1->x-p2->x)*fabs(p1->x-p2->x)+fabs(p1->y-p2->y)*fabs(p1->y-p2->y));
	}

sNum_t arc_len2(sPt_t *p2_1, sPt_t *oc, sNum_t or, sPt_t *p2_3){
    sVec_t v1, v3;
    sNum_t d, c;

    if(fabs(or) < LOW_THR)
        return 0.;

    convPts2Vec(oc, p2_1, &v1);
    convPts2Vec(oc, p2_3, &v3);

    dotVecs(&v1, &v3, &d);
    crossVecs(&v1, &v3, &c);

    d = d/(or*or);
    // d must be between -1 and 1 but because we do not use the true length of v1 and v3
    // (we use r instead to avoid some heavy calculations) it may be a little outside of this interval
    // so, let's just be sure we stay in this interval for acos to give a result
    if(d > 1.) {
        d = 1.;
    }
    else if(d < -1.) {
        d = -1.;
    }

    d = acos(d);

    if(or > 0.) {  // clock wise
        if(c > 0) {
            d = 2*M_PI - d;
        }
    }
    else {  // counter clock wise
        if(c < 0) {
            d = 2*M_PI - d;
        }
    }

    return fabs(d*or);
}


void path_len(sTrajEl_t tab[], int size){
	int i;

	for(i=0; i<size-1 ; i++){
		tab[i].arc_len=arc_len2(&(tab[i].p2), &tab[i].obs.c, tab[i].obs.r, &(tab[i+1].p1));
		tab[i].seg_len=seg_len(&(tab[i].p1), &(tab[i].p2));
		}
	tab[size-1].arc_len=0;
	}
