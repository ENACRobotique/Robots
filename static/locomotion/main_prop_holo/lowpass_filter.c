/**
 * lowpass_filter.c
 *
 * author: Ludovic Lacoste
 */

#include "lowpass_filter.h"

void lp_init(lowpass_t *lp, unsigned char shift) {
    lp->shift = shift;

    lp_reset(lp);
}

void lp_reset(lowpass_t *lp) {
    lp->reg = 0;
}

void lp_update(lowpass_t *lp, int32_t input) {
    lp->reg = lp->reg - (lp->reg >> lp->shift) + input;
}

int32_t lp_get(lowpass_t* lp) {
    return lp->reg >> lp->shift;
}

int64_t lp_getFullRes(lowpass_t* lp) {
    return lp->reg;
}
