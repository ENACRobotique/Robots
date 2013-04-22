#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "error.h"

#include "math_types.h"
#include "math_ops.h"
#include "tools.h"

sCcl_t list[] = {
    {10., 10., 0.},  // start point (current position)
    {90., 20., 20.},
    {110., 60., 20.},
//    {120., 30., 10.},
//    {110., 40., 15.},
    {210., 10., 0.}, // end point (goal)
//    {110., 83., 5.}   // check_arc test
};

ERROR main(int argc, char *argv[]) {
    unsigned int n = sizeof(list)/sizeof(*list);
    unsigned char *tab = NULL;
    int i, j, ok;
    sTgts_t tmp;

    // entry point
    printf("n=%u\n", n);    // number of elements

    // alloc and init possible links matrix
    tab = (unsigned char *)malloc(4*n*n*sizeof(unsigned char));
    RET_IF_NOT_(tab, ERR_MEM);
    for(i=0; i<4*n*n; i++) {
        tab[i] = 0;
    }

    // fill
    for(i=0; i<n; i++) {
        for(j=i+1; j<n; j++) {
            printf("step: obstacles %u and %u\n", i, j);

            get_tangents(&list[i], &list[j], &tmp);

            printf("  %u common tangents\n", tmp.nb);
            printf("  dist %.2f\n", tmp.O1O2);

            if(tmp.nb > 0) {
                check_segment(list, n, i, j, &tmp.s1, &ok);

                tab[(j*2) + (i*2)*2*n] = ok;
                tab[(i*2) + (j*2)*2*n] = ok;

                printf("    s1:%s [(%.2f,%.2f);(%.2f,%.2f)]\n", ok?"ok":"ko", tmp.s1.p1.x, tmp.s1.p1.y, tmp.s1.p2.x, tmp.s1.p2.y);
            }
            if(tmp.nb > 1) {
                check_segment(list, n, i, j, &tmp.s2, &ok);

                tab[(j*2+1) + (i*2+1)*2*n] = ok;
                tab[(i*2+1) + (j*2+1)*2*n] = ok;

                printf("    s2:%s [(%.2f,%.2f);(%.2f,%.2f)]\n", ok?"ok":"ko", tmp.s2.p1.x, tmp.s2.p1.y, tmp.s2.p2.x, tmp.s2.p2.y);
            }
            if(tmp.nb > 2) {
                check_segment(list, n, i, j, &tmp.s3, &ok);

                tab[(j*2) + (i*2+1)*2*n] = ok;
                tab[(i*2+1) + (j*2)*2*n] = ok;

                printf("    s3:%s [(%.2f,%.2f);(%.2f,%.2f)]\n", ok?"ok":"ko", tmp.s3.p1.x, tmp.s3.p1.y, tmp.s3.p2.x, tmp.s3.p2.y);
            }
            if(tmp.nb > 3) {
                check_segment(list, n, i, j, &tmp.s4, &ok);

                tab[(j*2+1) + (i*2)*2*n] = ok;
                tab[(i*2) + (j*2+1)*2*n] = ok;

                printf("    s4:%s [(%.2f,%.2f);(%.2f,%.2f)]\n", ok?"ok":"ko", tmp.s4.p1.x, tmp.s4.p1.y, tmp.s4.p2.x, tmp.s4.p2.y);
            }
        }
    }

    // show
    printf("possible links matrix:\n  ");
    for(i=0; i<2*n; i++) {
        printf(" %u%c", (i>>1), list[i>>1].r?(i&1?'b':'a'):' ');
    }
    printf("\n");
    for(i=0; i<2*n; i++) {
        printf("%u%c", (i>>1), list[i>>1].r?(i&1?'b':'a'):' ');
        for(j=0; j<2*n; j++) {
            printf("%c%c ", j?',':' ', tab[j+i*2*n]?'1':' ');
        }
        printf("\n");
    }

    // check_arc & arc_len tests
    {
        sPt_t p2_1, p2_2, c;
        sNum_t l;
        int dir = 0; // 0:clockwise (a) ; 1:counterclockwise (b)

        // checking if following arc from point 0 to point 3 around obstacle 2 is possible
        get_tangents(&list[0], &list[2], &tmp);
        p2_1 = tmp.s1.p2;

        get_tangents(&list[2], &list[3], &tmp);
        p2_2 = tmp.s1.p1;

        check_arc(list, n, 0, 3, 2, &p2_1, &p2_2, dir, &ok);

        printf("ok=%u\n", ok);

        // getting length of this arc
        c.x = list[2].x;
        c.y = list[2].y;

        arc_len(&p2_1, &c, &p2_2, list[2].r, dir, &l);

        printf("l=%.2f\n", l);
    }

	return 0;
}

