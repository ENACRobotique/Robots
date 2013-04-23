#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "error.h"

#include "math_types.h"
#include "math_ops.h"
#include "tools.h"

ERROR main(int argc, char *argv[]) {
    uint8_t i, j, ok;

    // entry point
    printf("N=%u\n", N);    // number of elements

    // fill
    get_links();

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

    // check_arc & arc_len tests
    {
        sPt_t p2_1, p2_2;
        sNum_t l;
        int dir = 0; // 0:clockwise (a) ; 1:counterclockwise (b)

        // checking if following arc from point 0 to point 3 around obstacle 2 is possible (clockwise external => s1)
        p2_1 = tgts[0][2].s1.p2;
        p2_2 = tgts[2][3].s1.p1;

        ok = check_arc(0, &p2_1, 2, 3, &p2_2, dir);

        printf("ok=%u\n", ok);

        // getting length of this arc
        l = arc_len(&p2_1, 2, &p2_2, dir);

        printf("l=%.2f\n", l);
    }

	return 0;
}

