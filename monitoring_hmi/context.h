#ifndef _CONTEXT_H
#define _CONTEXT_H

#include "draw_list.h"

typedef struct {
    int verbose;

    // trajectory data
    int tid;

    // robot data
    bn_Address prop_address;
    sPosList poslist;
    sTrajList trajlist;

    // position media (first tab in gui)
    int pos_mid;
    unsigned int pos_szx;
    unsigned int pos_szy;
    int pos_cur;
    unsigned char *pos_data[2];

    // event data
    int mouse_event;
    int mouse_x;
    int mouse_y;
} context_t;

#endif

