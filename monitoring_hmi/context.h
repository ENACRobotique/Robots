#ifndef _CONTEXT_H
#define _CONTEXT_H

typedef struct {
    int verbose;

    // trajectory data
    int tid;

    // position media
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

