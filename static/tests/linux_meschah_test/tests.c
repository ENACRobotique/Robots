/**************************************************************************
 **
 ** Copyright (C) 1993 David E. Steward & Zbigniew Leyk, all rights reserved.
 **
 **			     Meschach Library
 **
 ** This Meschach Library is provided "as is" without any express
 ** or implied warranty of any kind with respect to this software.
 ** In particular the authors shall not be liable for any direct,
 ** indirect, special, incidental or consequential damages arising
 ** in any way from use of the software.
 **
 ** Everyone is granted permission to copy, modify and redistribute this
 ** Meschach Library, provided:
 **  1.  All copies contain this copyright notice.
 **  2.  All modified copies shall carry a notice stating who
 **      made the last modification and the date of such modification.
 **  3.  No charge is made for this software or works derived from it.
 **      This clause shall not be construed as constraining other software
 **      distributed on the same medium as this software, nor is a
 **      distribution fee considered a charge.
 **
 ***************************************************************************/

#include <math.h>
#include <matrix.h>
#include <matrix2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

double dclean(double x){
    static double y;
    y = x;
    return y; /* prevents optimization */
}

float fclean(float x){
    static float y;
    y = x;
    return y; /* prevents optimization */
}

void test_macheps(){
    static double deps;
    static float feps;

    deps = 1.0;
    feps = 1.0;

    while (dclean(1.0 + deps) > 1.0)
        deps = 0.5 * deps;

    while (fclean(1.0 + feps) > 1.0)
        feps = 0.5 * feps;

    printf("double: %g\n", 2.0 * deps);
    printf("float : %g\n", 2.0 * feps);
}

void test_linearsolve(){
    MAT *A = MNULL;
    MAT *Am1 = NULL;
    VEC *b = VNULL;
    VEC *x = VNULL;
    VEC *res = VNULL;;

    // problem initialization
    A = m_get(2, 2);
    A->me[0][0] = 1;
    A->me[0][1] = 2;
    A->me[1][0] = 3;
    A->me[1][1] = 4;

    // inversion of A
    Am1 = m_inverse(A, Am1);

    printf("A:\n");
    m_output(A);
    printf("A^-1:\n");
    m_output(Am1);

    b = v_get(2);
    b->ve[0] = 1.;
    b->ve[1] = -2.;

    // calculation of A^-1 * b
    x = mv_mlt(Am1, b, x);

    printf("b:\n");
    v_output(b);
    printf("x:\n");
    v_output(x);

    // verification (calculates b - A*x)
    res = mv_mltadd(b, x, A, -1, res);

    printf("residu:\n");
    v_output(res);

    // free allocations
    M_FREE(A);
    M_FREE(Am1);
    V_FREE(b);
    V_FREE(x);
    V_FREE(res);
}

void test_errlinearsolve(){
    MAT *A = MNULL;
    MAT *Am1 = NULL;
    VEC *b = VNULL;
    VEC *x = VNULL;
    VEC *res = VNULL;;

    double max1_res = 0.;
    double max1_t = -1.;
    VEC *max1_x = VNULL;
    VEC *max1_b = VNULL;

    double max2_res_norm = 0.;
    double max2_t = -1.;
    VEC *max2_x = VNULL;
    VEC *max2_b = VNULL;

    // log init
    FILE *fcsv = NULL;//fopen("log.csv", "wb+");
    if(fcsv) fprintf(fcsv, "t, b0, b1, x0, x1, r0, r1, n\n");

    // problem initialization
    double angle = M_PI / 4.;
    A = m_get(2, 2);
    A->me[0][0] = cos(angle);
    A->me[0][1] = -sin(angle);
    A->me[1][0] = sin(angle);
    A->me[1][1] = cos(angle);

    // inversion of A
    Am1 = m_inverse(A, MNULL);

    printf("A:\n");
    m_output(A);
    printf("A^-1:\n");
    m_output(Am1);

    for(double t = 0.; t <= 1.; t+=0.001){
        b = v_resize(b, 2);
        b->ve[0] = cos(2.*M_PI*t);
        b->ve[1] = sin(2.*M_PI*t);

        // calculation of A^-1 * b
        x = mv_mlt(Am1, b, x);

//            printf("b:\n");
//            v_output(b);
//            printf("x:\n");
//            v_output(x);

        double res_norm = v_norm2(x) - 1.;
        if(fabs(res_norm) > max2_res_norm){
            max2_res_norm = fabs(res_norm);
            max2_x = v_copy(x, max2_x);
            max2_b = v_copy(b, max2_b);
            max2_t = t;
        }

        // verification (calculates b - A*x)
        res = mv_mltadd(b, x, A, -1, res);

//            printf("residu:\n");
//            v_output(res);

//        if(fcsv) fprintf(fcsv, "t, b0, b1, x0, x1, r0, r1, n\n");
        if(fcsv) fprintf(fcsv, "%g, %g, %g, %g, %g, %g, %g, %g\n", t, b->ve[0], b->ve[1], x->ve[0], x->ve[1], res->ve[0], res->ve[1], res_norm);

        if(fabs(res->ve[0]) > max1_res){
            max1_res = fabs(res->ve[0]);
            max1_x = v_copy(x, max1_x);
            max1_b = v_copy(b, max1_b);
            max1_t = t;
        }
        if(fabs(res->ve[1]) > max1_res){
            max1_res = fabs(res->ve[1]);
            max1_x = v_copy(x, max1_x);
            max1_b = v_copy(b, max1_b);
            max1_t = t;
        }
    }

    printf("max1_res=%g\n", max1_res);
    printf("max1_t  =%g\n", max1_t);
    printf("max1_b:\n");
    v_output(max1_b);
    printf("max1_x:\n");
    v_output(max1_x);

    printf("max2_res_norm=%g\n", max2_res_norm);
    printf("max2_t       =%g\n", max2_t);
    printf("max2_b:\n");
    v_output(max2_b);
    printf("max2_x:\n");
    v_output(max2_x);

    if(fcsv) fclose(fcsv);

    // free allocations
    M_FREE(A);
    M_FREE(Am1);
    V_FREE(b);
    V_FREE(x);
    V_FREE(res);
    V_FREE(max1_b);
    V_FREE(max1_x);
    V_FREE(max2_b);
    V_FREE(max2_x);
}

void test_matvecops(){
    double A_angle = M_PI / 6.;
    MAT *A = m_get(3, 3);
    MAT *Am1 = MNULL;

    double B_angle = - M_PI / 3.;
    MAT *B = m_get(3, 3);
    MAT *Bm1 = MNULL;

    VEC *b = v_get(3);

    MAT *mtmp = MNULL;
    VEC *vtmp = VNULL;
    VEC *ABb = VNULL;
    VEC *new_b = VNULL;

    A->me[0][0] = cos(A_angle);
    A->me[0][1] = -sin(A_angle);
    A->me[0][2] = 20.;
    A->me[1][0] = sin(A_angle);
    A->me[1][1] = cos(A_angle);
    A->me[1][2] = -15.;
    A->me[2][0] = 0.;
    A->me[2][1] = 0.;
    A->me[2][2] = 1.;

    printf("A:\n");
    m_output(A);

    Am1 = m_inverse(A, Am1);

    printf("A^-1:\n");
    m_output(Am1);

    B->me[0][0] = cos(B_angle);
    B->me[0][1] = -sin(B_angle);
    B->me[0][2] = -2.;
    B->me[1][0] = sin(B_angle);
    B->me[1][1] = cos(B_angle);
    B->me[1][2] = -5.;
    B->me[2][0] = 0.;
    B->me[2][1] = 0.;
    B->me[2][2] = 1.;

    printf("B:\n");
    m_output(B);

    Bm1 = m_inverse(B, Bm1);

    printf("B^-1:\n");
    m_output(Bm1);

    b->ve[0] = 1.5;
    b->ve[1] = -1.;
    b->ve[2] = 1.;

    printf("b:\n");
    v_output(b);

    mtmp = m_mlt(A, B, mtmp);
    ABb = mv_mlt(mtmp, b, ABb);

    printf("ABb:\n");
    v_output(ABb);

    vtmp = mv_mlt(Am1, ABb, vtmp);
    new_b = mv_mlt(Bm1, vtmp, new_b);

    printf("new_b:\n");
    v_output(new_b);

    M_FREE(A);
    M_FREE(B);
    V_FREE(b);
    M_FREE(mtmp);
    V_FREE(vtmp);
    V_FREE(new_b);
    V_FREE(ABb);
}

struct{
    void (*f)();
    char* s;
} tests[]={
        {test_macheps, "Machine epsilon test (double & float)"},
        {test_linearsolve, "Ax=b solutions (using explicit matrix inversion"},
        {test_errlinearsolve, "Solves Ax=b for different b and check residual (using explicit matrix inversion"},
        {test_matvecops, "Matrix vector operations"},
};

int main() {
    char* buf = NULL;
    for(int i = 0; i < sizeof(tests)/sizeof(*tests); i++){
        const char format[] = "# Starting test #%02i \"%s\" #\n";

        // create border
        {
            int sz = sizeof(format) - 2 + strlen(tests[i].s) + 1 - 2 - 2;
            buf = realloc(buf, sz);
            int j;
            for(j = 0; j < sz-1; j++){
                buf[j] = '#';
            }
            buf[j] = '\0';
        }

        if(i){
            puts("\n");
        }
        puts(buf);
        printf(format, i, tests[i].s);
        puts(buf);

        // call test
        tests[i].f();

        puts(buf);
        printf("# End of test #%02i   \"%s\" #\n", i, tests[i].s);
        puts(buf);
    }
    if(buf) free(buf);

//    malloc_stats();

    return EXIT_SUCCESS;
}
