/**
 * median_filter.c
 *
 * author: Ludovic Lacoste
 */

//#include <malloc.h>
#include <stdlib.h>

#include "median_filter.h"

void mf_init(median_t *mf, unsigned char nb, int def) {
    int i;
    mf->nb = nb;

    mf->values = malloc(mf->nb * sizeof(*mf->values));
    mf->indexes = malloc(mf->nb * sizeof(*mf->indexes));

    mf_reset(mf);

    for(i = 0; i < nb; i++) {
    	mf->values[i] = def;
    }
}

void mf_deinit(median_t* mf) {
    if(mf->values) {
        free(mf->values);
    }

    if(mf->indexes) {
        free(mf->indexes);
    }
}

void mf_reset(median_t *mf) {
    unsigned char i;
    for(i = 0; i < mf->nb; i++){
        mf->indexes[i] = i;
        mf->values[i] = 0;
    }

    mf->currIdx = 0;
}

void mf_update(median_t *mf, int input) {
    // replace last value
    mf->values[mf->currIdx] = input;
    mf->currIdx = (mf->currIdx + 1) % mf->nb;

    // sort array (insertion sort)
    int i, j, k;
    for(i = 1; i < mf->nb; i++){
        k = mf->indexes[i];
        j = i;
        while(j > 0 && mf->values[mf->indexes[j - 1]] > mf->values[k]){
            mf->indexes[j] = mf->indexes[j - 1];
            j--;
        }
        mf->indexes[j] = k;
    }
}

int mf_get(median_t* mf) {
    return mf->values[mf->indexes[mf->nb/2]];
}

