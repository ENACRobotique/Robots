#ifndef _DRAW_LIST_H
#define _DRAW_LIST_H

#include "messages.h"

// list of previous positions

typedef struct sPosLEl sPosLEl;
struct sPosLEl{
    sPosPayload pos;
//    unsigned int t; // time
    struct sPosLEl *next;
};

typedef struct{
    sPosLEl *head;
    sPosLEl *tail;
    unsigned int len;
    int maxlen;

    sPosLEl *curr; // used to iterate
} sPosList;

void    pl_init     (sPosList *pl, int maxlen); // maxlen=-1 if no limit
int     pl_addTail  (sPosList *pl, sPosPayload *p);
sPosLEl *pl_getFirst (sPosList *pl);
sPosLEl *pl_getNext  (sPosList *pl);

// list of trajectories sent to the robot

typedef struct sTrajLEl sTrajLEl;
struct sTrajLEl{
    sTrajElRaw_t traj;
//    unsigned int t; // time
    struct sTrajLEl *next;
};

typedef struct{
    sTrajLEl *head;
    sTrajLEl *tail;
    unsigned int len;
    int maxlen;

    sTrajLEl *curr; // used to iterate
} sTrajList;

void        tl_init     (sTrajList *pl, int maxlen); // maxlen=-1 if no limit
int         tl_addTail  (sTrajList *pl, sTrajElRaw_t *p);
sTrajLEl *  tl_getFirst (sTrajList *pl);
sTrajLEl *  tl_getNext  (sTrajList *pl);

#endif
