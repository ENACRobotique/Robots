/**
 * lowpass_filter.h
 *
 * author: Ludovic Lacoste
 */

#ifndef _LOWPASS_FILTER_H
#define _LOWPASS_FILTER_H

#include <stdint.h>

typedef struct {
    unsigned char shift;

    int64_t reg;
} lowpass_t;

void        lp_init         (lowpass_t *lp, unsigned char shift);
void        lp_reset        (lowpass_t *lp);
void        lp_update       (lowpass_t *lp, int32_t input);
int32_t     lp_get          (lowpass_t* lp);
int64_t     lp_getFullRes   (lowpass_t* lp); // shift it right

#endif
