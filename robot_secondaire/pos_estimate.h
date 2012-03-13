#ifndef _POS_ESTIMATE_H
#define _POS_ESTIMATE_H

typedef enum {
    pLOST,

    pLEFT,
    pLEFT_CENTER,
    pCENTER,
    pRIGHT_CENTER,
    pRIGHT,

    pNUM,

    pWTF = 0x80 // BIT(7)
} ePosition;
#define POS(p) ((p)&~pWTF)

extern const int pos_next[pNUM /* prev position */][2 /* left sensor */][2 /* right sensor */];

#endif

