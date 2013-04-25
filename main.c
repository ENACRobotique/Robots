#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "error.h"

#include "math_types.h"
#include "math_ops.h"
#include "tools.h"
#include "a_star.h"

ERROR main(int argc, char *argv[]) {
    uint8_t i, j;
    iABObs_t *path, *p, prev;

    // entry point
    printf("N=%u\n", N);    // number of elements

    printf("sizeof(obs) =%uB\n", sizeof(obs));
    printf("sizeof(tgts)=%uB\n", sizeof(tgts));
    printf("sizeof(lnk) =%uB\n", sizeof(lnk));

    printf("\n");

    // fill
    fill_tgts_lnk();

    // show
    printf("possible links matrix:\n  ");
    for(i=0; i<2*N; i++) {
        printf(" %u%c", (i>>1), obs[i>>1].r?(i&1?'b':'a'):' ');
    }
    printf("\n");
    for(i=0; i<2*N; i++) {
        printf("%u%c", (i>>1), obs[i>>1].r?(i&1?'b':'a'):' ');
        for(j=0; j<2*N; j++) {
            printf("%c%c ", j?',':' ', lnk[i][j]?'1':' ');
        }
        printf("\n");
    }

    // A* test
    printf("\n\nA* test\n");
    path = a_star(A(0), A(N-1));
    printf("path from 0 to %u:\n", N-1);
    if(path) {
        p = path;
        for(; *p!=NOELT; p++) {
            printf("  obs %u%c\n", O(*p), DIR(*p)?'b':'a');
        }
    }

    // builds trajectory for robot
    printf("trajectory data:\n");
    if(path) {
        p = path;
        prev = NOELT;
        for(; *p!=NOELT; prev = *p, p++) {
            if(prev == NOELT)
                continue;

            sSeg_t *s = tgt(prev, *p);
            printf("  {isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f)},\n", s->p1.x, s->p1.y, s->p2.x, s->p2.y, obs[O(*p)].c.x, obs[O(*p)].c.y, obs[O(*p)].r);
        }
    }

    // builds arbitrary trajectory
    printf("arbitrary trajectory data:\n");
    iABObs_t p2[] = {A(0), B(1), A(2), B(3), A(4), NOELT};
    p = p2;
    prev = NOELT;
    for(; *p!=NOELT; prev = *p, p++) {
        if(prev == NOELT)
            continue;

        sSeg_t *s = tgt(prev, *p);
        printf("  {isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f)},\n", s->p1.x, s->p1.y, s->p2.x, s->p2.y, obs[O(*p)].c.x, obs[O(*p)].c.y, obs[O(*p)].r);
    }

	return 0;
}

