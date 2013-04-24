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
    iABObs_t *path;

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
    path = a_star(A(0), A(5));
    printf("path from 0 to 5:\n");
    if(path)
        for(; *path!=NOELT; path++) {
            printf("  obs %u%c\n", O(*path), DIR(*path)?'b':'a');
        }

	return 0;
}

