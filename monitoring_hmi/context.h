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
    float pos_maxx;
    float pos_maxy;
    float pos_wfactor;
    int pos_cur;
    unsigned char *pos_data[2];

    // event data
    int mouse_event;
    int mouse_x;
    int mouse_y;
} context_t;

#define CTX_X_CM2PX(c, x) ((int)((c)->pos_wfactor*(float)(x) + 0.5))
#define CTX_Y_CM2PX(c, y) ((int)((c)->pos_wfactor*((c)->pos_maxy - (float)(y)) + 0.5))
#define CTX_DX_CM2PX(c, dx) ((int)((c)->pos_wfactor*(float)(dx) + 0.5))
#define CTX_DY_CM2PX(c, dy) ((int)((c)->pos_wfactor*(-(float)(dy)) + 0.5))
#define CTX_CM2PX(c, r) CTX_DX_CM2PX(c, r)
#define CTX_X_PX2CM(c, x) ((float)(x)/(c)->pos_wfactor)
#define CTX_Y_PX2CM(c, y) ((c)->pos_maxy - (float)(y)/(c)->pos_wfactor)

#define CTX_WIDTH(c) (CTX_X_CM2PX(c, (c)->pos_maxx)+1)
#define CTX_ROWSTRIDE(c) ((CTX_X_CM2PX(c, (c)->pos_maxx)+1)*3)
#define CTX_HEIGHT(c) (CTX_Y_CM2PX(c, 0)+1)
#define CTX_BUFSZ(c) (CTX_ROWSTRIDE(c)*CTX_HEIGHT(c))

#endif

