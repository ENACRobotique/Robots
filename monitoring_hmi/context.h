#ifndef _CONTEXT_H
#define _CONTEXT_H

typedef struct {
    int verbose;

    // position media
    int pos_mid;
    unsigned int pos_szx;
    unsigned int pos_szy;
    int pos_cur;
    unsigned char *pos_data[2];
} context_t;

#endif

