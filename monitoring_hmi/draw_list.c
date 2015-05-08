#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <string.h>

#include "draw_list.h"

// function for list of positions

void pl_init(sPosList *pl, int maxlen){
    assert(pl);

    pl->head = NULL;
    pl->tail = NULL;
    pl->len = 0;
    pl->maxlen = maxlen;
    pl->curr = NULL;
}

int pl_addTail(sPosList *pl, sGenericPosStatus *p){
    sPosLEl *el;
    assert(pl);

    el = (sPosLEl *)malloc(sizeof(sPosLEl));
    if(!el){
        errno = ENOMEM;
        return -1;
    }

    el->next = NULL;
    if(pl->tail){
        assert(pl->head && pl->len > 0);

        pl->tail->next = el;
        pl->tail = el;
    }
    else{
        assert(!pl->head && !pl->len);

        pl->head = pl->tail = el;
    }
    pl->len++;

    if(p)
        memcpy(&el->pos, p, sizeof(el->pos));
    else
        memset(&el->pos, 0, sizeof(el->pos));

    if(pl->maxlen > 0 && pl->len > pl->maxlen){
        el = pl->head;
        pl->head = pl->head->next;
        if(pl->curr == el){
            pl->curr = NULL;
        }
        pl->len--;

        free(el);
    }

    return 0;
}

sPosLEl *pl_getFirst(sPosList *pl){
    assert(pl);

    pl->curr = pl->head;

    return pl->curr;
}

sPosLEl *pl_getNext(sPosList *pl){
    assert(pl && pl->curr);

    pl->curr = pl->curr->next;

    return pl->curr;
}

// functions for list of trajectories (assumes consecutive, increasing tids)

void tl_init(sTrajList *pl, int maxlen){
    assert(pl);

    pl->head = NULL;
    pl->tail = NULL;
    pl->len = 0;
    pl->maxlen = maxlen;
    pl->curr = NULL;
}

int tl_addTail(sTrajList *tl, sTrajElRaw_t *p){
    sTrajLEl *el;
    assert(tl);

    el = (sTrajLEl *)malloc(sizeof(sTrajLEl));
    if(!el){
        errno = ENOMEM;
        return -1;
    }

    el->next = NULL;
    if(tl->tail){
        assert(tl->head && tl->len > 0);

        tl->tail->next = el;
        tl->tail = el;
    }
    else{
        assert(!tl->head && !tl->len);

        tl->head = tl->tail = el;
    }
    tl->len++;

    if(p)
        memcpy(&el->traj, p, sizeof(el->traj));
    else
        memset(&el->traj, 0, sizeof(el->traj));

    if(tl->maxlen > 0){
        while(tl->head->traj.tid <= tl->tail->traj.tid - tl->maxlen){
            el = tl->head;
            tl->head = tl->head->next;
            if(tl->curr == el){
                tl->curr = NULL;
            }
            tl->len--;

            free(el);
        }
    }

    return 0;
}

sTrajLEl *tl_getFirst(sTrajList *tl){
    assert(tl);

    tl->curr = tl->head;

    return tl->curr;
}

sTrajLEl *tl_getNext(sTrajList *tl){
    assert(tl && tl->curr);

    tl->curr = tl->curr->next;

    return tl->curr;
}
