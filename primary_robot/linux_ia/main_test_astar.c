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
//    sTrajEl_t *p, prev;
//    sTrajEl_t* traj = NULL;
    sPath_t path;

    // entry point
    printf("N=%u\n", N);    // number of elements

    printf("sizeof(obs) =%luB\n", sizeof(obs));
    printf("sizeof(tgts)=%luB\n", sizeof(tgts));
    printf("sizeof(lnk) =%luB\n", sizeof(lnk));

    printf("\n");

    // fill
    fill_tgts_lnk();

    // show
    printf("possible links matrix:\n  ");
    for(i=0; i<2*N; i++) {
        printf(" %u%c", (i>>1)%10, obs[i>>1].r?(i&1?'b':'a'):' ');
    }
    printf("\n");
    for(i=0; i<2*N; i++) {
        printf("%u%c", (i>>1)%10, obs[i>>1].r?(i&1?'b':'a'):' ');
        for(j=0; j<2*N; j++) {
            printf("%c%c ", j?',':' ', lnk[i][j]?'1':' ');
        }
        printf("\n");
    }

    // A* test
    printf("\n\nA* test\n");
    a_star(A(0), A(N-1), &path);
    printf("path from 0a to %ua (dist %.2fcm):\n", N-1, path.dist);
    if(path.path)
        for(i = 0; i < path.path_len; i++) {
            printf("  %u: p1 x%f y%f, p2 x%f y%f, obs x%f y%f r%.2f, a_l%f s_l%f\n", i, path.path[i].p1.x, path.path[i].p1.y, path.path[i].p2.x, path.path[i].p2.y,path.path[i].obs.c.x,path.path[i].obs.c.y, path.path[i].obs.r,path.path[i].arc_len,path.path[i].seg_len);
        }

    printf("\n\nA* test\n");
    a_star(A(N-1), A(0), &path);
    printf("path from %ua to 0a (dist %.2fcm):\n", N-1, path.dist);
    if(path.path)
        for(i = 0; i < path.path_len; i++) {
            printf("  %u: p1 x%f y%f, p2 x%f y%f, obs x%f y%f r%.2f, a_l%f s_l%f\n", i, path.path[i].p1.x, path.path[i].p1.y, path.path[i].p2.x, path.path[i].p2.y,path.path[i].obs.c.x,path.path[i].obs.c.y, path.path[i].obs.r,path.path[i].arc_len,path.path[i].seg_len);
       }

#if 0
    printf("\n\nA* test\n");
    a_star(A(0), A(N-1), &path);
    printf("path from 0a to %ua (dist %.2fcm):\n", N-1, path.dist);
    if(path.path)
        for(i = 0; i < path.path_len; i++) {
            printf("  {isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f)},\n", path.path[i].p1.x, path.path[i].p1.y, path.path[i].p2.x, path.path[i].p2.y, path.path[i].seg_len, path.path[i].obs.c.x, path.path[i].obs.c.y, path.path[i].obs.r, path.path[i].arc_len, 40*sqrt(path.path[i].obs.r));
        }
#endif

// builds arbitrary trajectory for robot (blue side)
    {
        obs[0].c.x = 7.5;
        obs[0].c.y = 100.;
        obs[0].r = 0.;

        obs[2].r = 8.;
        obs[3].r = 8.;

        obs[N-1].c.x = 20.;
        obs[N-1].c.y = 100.;
        obs[N-1].r = 0.;

        fill_tgts_lnk();

        sSeg_t *s1 = tgt(A(0), A(3));
            sNum_t s1_len; distPt2Pt(&s1->p1, &s1->p2, &s1_len);
            sNum_t a1_len = arc_len(A(0), A(3), B(2));
        sSeg_t *s2 = tgt(A(3), B(2));
            sNum_t s2_len; distPt2Pt(&s2->p1, &s2->p2, &s2_len);
            sNum_t a2_len = arc_len(A(3), B(2), A(N-1));
        sSeg_t *s3 = tgt(B(2), A(N-1));
            sNum_t s3_len; distPt2Pt(&s3->p1, &s3->p2, &s3_len);

        printf("blue side:\n");
        printf("  {isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f)},\n", s1->p1.x, s1->p1.y, s1->p2.x, s1->p2.y, s1_len, obs[3].c.x, obs[3].c.y, obs[3].r, a1_len);
        printf("  {isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f)},\n", s2->p1.x, s2->p1.y, s2->p2.x, s2->p2.y, s2_len, obs[2].c.x, obs[2].c.y, obs[2].r, a2_len);
        printf("  {isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f)},\n", s3->p1.x, s3->p1.y, s3->p2.x, s3->p2.y, s3_len, obs[N-1].c.x, obs[N-1].c.y, obs[N-1].r, 0.);
    }

// builds arbitrary trajectory for robot (red side)
    {
        obs[0].c.x = 300.-7.5;
        obs[0].c.y = 100.;
        obs[0].r = 0.;

        obs[8].r = 8.;
        obs[9].r = 8.;

        obs[N-1].c.x = 300.-20.;
        obs[N-1].c.y = 100.;
        obs[N-1].r = 0.;

        fill_tgts_lnk();

        sSeg_t *s1 = tgt(A(0), B(9));
            sNum_t s1_len; distPt2Pt(&s1->p1, &s1->p2, &s1_len);
            sNum_t a1_len = arc_len(A(0), B(9), B(8));
        sSeg_t *s2 = tgt(B(9), B(8));
            sNum_t s2_len; distPt2Pt(&s2->p1, &s2->p2, &s2_len);
            sNum_t a2_len = arc_len(B(9), B(8), A(N-1));
        sSeg_t *s3 = tgt(B(8), A(N-1));
            sNum_t s3_len; distPt2Pt(&s3->p1, &s3->p2, &s3_len);

        printf("red side:\n");
        printf("  {isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f)},\n", s1->p1.x, s1->p1.y, s1->p2.x, s1->p2.y, s1_len, obs[9].c.x, obs[9].c.y, obs[9].r, a1_len);
        printf("  {isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f)},\n", s2->p1.x, s2->p1.y, s2->p2.x, s2->p2.y, s2_len, obs[8].c.x, obs[8].c.y, obs[8].r, a2_len);
        printf("  {isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f), isD2I(%.2f)},\n", s3->p1.x, s3->p1.y, s3->p2.x, s3->p2.y, s3_len, obs[N-1].c.x, obs[N-1].c.y, obs[N-1].r, 0.);
    }

    return 0;
}

