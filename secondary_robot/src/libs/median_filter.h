/**
 * median_filter.h
 *
 * author: Ludovic Lacoste
 */

#ifndef _MEDIAN_FILTER_H
#define _MEDIAN_FILTER_H

#include <stdint.h>

typedef struct {
    int nb;

    int* values; // circular buffer
    int* indexes; // order of the values
    int currIdx; // index where you insert the next value in the circular buffer
} median_t;

void        mf_init         (median_t *mf, unsigned char nb, int def);
void        mf_deinit       (median_t* mf);
void        mf_reset        (median_t *mf);
void        mf_update       (median_t *mf, int input);
int         mf_get          (median_t* mf);

#endif

