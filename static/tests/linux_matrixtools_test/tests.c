#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/time.h>

#include "mt_io.h"
#include "mt_mat.h"
#include "mt_vec.h"

#define M_PI (3.141592653589793)

#define dMSHIFT ((double)(1 << MAT_SHIFT))
#define dVSHIFT ((double)(1 << VEC_SHIFT))
#define m_init(m, r, c) mt_m_init((m), (r), (c), MAT_SHIFT)
#define M_INITS(r, c) MT_M_INITS((r), (c), MAT_SHIFT)
#define v_init(v, e) mt_v_init((v), (e), VEC_SHIFT)
#define V_INITS(e) MT_V_INITS((e), VEC_SHIFT)

void test_machtypes() {
    printf("sizeof(int)=%zu\n", sizeof(int));
    printf("sizeof(long int)=%zu\n", sizeof(long int));
    printf("sizeof(long long int)=%zu\n", sizeof(long long));
    printf("sizeof(int*)=%zu\n", sizeof(int*));
    printf("sizeof(MT_VEC)=%zu\n", sizeof(MT_VEC));
    printf("sizeof(MT_MAT)=%zu\n", sizeof(MT_MAT));
}

void test_matrixadd() {
#define MAT_SHIFT (20)

    int ret;

    MT_MAT A = M_INITS(2, 2);
    MT_MAT B = M_INITS(2, 2);
    MT_MAT ApB = M_INITS(2, 2);

    MT_M_AT(&A, 0, 0) =  M_PI * dMSHIFT;
    MT_M_AT(&A, 0, 1) = -2.   * dMSHIFT;
    MT_M_AT(&A, 1, 0) =  3.   * dMSHIFT;
    MT_M_AT(&A, 1, 1) =  1.5  * dMSHIFT;

    MT_M_AT(&B, 0, 0) = -M_PI * dMSHIFT;
    MT_M_AT(&B, 0, 1) =  2.   * dMSHIFT;
    MT_M_AT(&B, 1, 0) = -3.   * dMSHIFT;
    MT_M_AT(&B, 1, 1) = -1.5  * dMSHIFT;

    ret = mt_mm_add(&A, &B, &ApB);
    assert(!ret);

    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 2; j++) {
            assert(MT_M_AT(&ApB, i, j) < 1);
        }
    }

#undef MAT_SHIFT
}

void test_linearsolve() {
#define MAT_SHIFT (16)
#define VEC_SHIFT (16)

    int ret;

    // containers initialization, via function call:  (internal call to malloc(), be sure to call mt_*_free() when done)
    MT_MAT A;       m_init(&A, 2, 2);
    MT_MAT Am1;     m_init(&Am1, 2, 2);
    MT_VEC b;       v_init(&b, 2);
    // or via constant initializer on stack:  (you don't need to call mt_*_free() on those objects but the memory where it points will be released at the end of this function)
    MT_MAT AAm1   = M_INITS(2, 2);
    MT_VEC x      = V_INITS(2);
    MT_VEC res    = V_INITS(2);

    printf("b  : %p (heap)\n", b.ve);
    printf("x  : %p (stack)\n", x.ve);
    printf("res: %p (stack)\n", res.ve);

    // problem initialization
    MT_M_AT(&A, 0, 0) =  M_PI * dMSHIFT;
    MT_M_AT(&A, 0, 1) = -2.   * dMSHIFT;
    MT_M_AT(&A, 1, 0) =  3.   * dMSHIFT;
    MT_M_AT(&A, 1, 1) =  1.5  * dMSHIFT;

    // inversion of A
    ret = mt_m_inv(&A, &Am1);
    assert(!ret);

    // computes A*A^-1 to verify we get the identity
    ret = mt_mm_mlt(&A, &Am1, &AAm1);
    assert(!ret);

    printf("A, ");
    mt_m_output(&A);
    printf("A^-1, ");
    mt_m_output(&Am1);
    printf("A*A^-1, ");
    mt_m_output(&AAm1);

    MT_V_AT(&b, 0) =  1. * dVSHIFT;
    MT_V_AT(&b, 1) = -2. * dVSHIFT;

    // calculation of A^-1 * b
    ret = mt_mv_mlt(&Am1, &b, &x);
    assert(!ret);

    printf("b, ");
    mt_v_output(&b);
    printf("x, ");
    mt_v_output(&x);

    // verification (calculates b - A*x)
    ret = mt_mv_mltadd(&b, -1<<VEC_SHIFT, &A, &x, &res);
    assert(!ret);

    printf("residu, ");
    mt_v_output(&res);

    // shift-independent check of precision (must be true for any shift)
    assert(abs(res.ve[0]) <= 5); // error up to 5 LSB tolerated given the number of operations
    assert(abs(res.ve[1]) <= 5);

    // free allocations
    mt_m_free(&A);
    mt_m_free(&Am1);
    mt_v_free(&b);
    // calls to mt_*_free() do nothing in case of stack-allocated object, you can omit those next calls
    mt_m_free(&AAm1);
    mt_v_free(&x);
    mt_v_free(&res);

#undef MAT_SHIFT
#undef VEC_SHIFT
}

void test_invmatrix() {
#define MAT_SHIFT (27)

#define RAD_SHIFT (13)
// MAT over RAD shift
#if MAT_SHIFT > RAD_SHIFT
#define dMoRSHIFT ((double)(1 << (MAT_SHIFT - RAD_SHIFT)))
#else
#define dMoRSHIFT ((double)(1 >> (RAD_SHIFT - MAT_SHIFT)))
#endif
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif
#define PI (M_PI)
#define WDIAM (3.25*2.54)  // wheel diameter (cm)
#define IpR (500.*676./49.) // increments per revolution (6897.959183673)
#define DpR (PI*WDIAM) // distance per revolution (25.933847355 cm)
#define D2I(d) ((d)*IpR/DpR) // (265.982871309 increments per centimeter)

    int ret;

    const int32_t mat_rob2pods[3][3] = {
    	    {-0.5 * dMSHIFT,  0.866025403784439 * dMSHIFT, D2I(15.5) * dMoRSHIFT},
    	    {-0.5 * dMSHIFT, -0.866025403784439 * dMSHIFT, D2I(15.5) * dMoRSHIFT},
    	    { 1.  * dMSHIFT,  0                 * dMSHIFT, D2I(15.5) * dMoRSHIFT}
    };

    // static allocation on stack matrices (no need to free those)
    MT_MAT M_rob2pods = M_INITS(3, 3);
    MT_MAT M_pods2rob = M_INITS(3, 3);
    MT_MAT M_product = M_INITS(3, 3);

    // init input matrix
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            MT_M_AT(&M_rob2pods, i, j) = mat_rob2pods[i][j];
        }
    }

//#define TIME_PERFS

#ifdef TIME_PERFS
    struct timeval time;
    gettimeofday(&time, 0);
    long start_time = 1000000 * time.tv_sec + time.tv_usec;

    for(int i = 0; i < 1000000; i++){
#endif
        ret = mt_m_inv(&M_rob2pods, &M_pods2rob);
        assert(!ret);

        ret = mt_mm_mlt(&M_rob2pods, &M_pods2rob, &M_product);
        assert(!ret);
#ifdef TIME_PERFS
    }

    gettimeofday(&time, 0);
    long cur_time = 1000000 * time.tv_sec + time.tv_usec;

    printf("duration: %li\n", cur_time - start_time);
#else
    printf("M_rob2pods, ");
    mt_m_output(&M_rob2pods);
    printf("M_pods2rob, ");
    mt_m_output(&M_pods2rob);
    printf("M_product, ");
    mt_m_output(&M_product);
#endif

#undef MAT_SHIFT
#undef RAD_SHIFT
#undef WDIAM
#undef IpR
}

void test_mtdm_square() {
#define MAT_SHIFT (27)
#define VEC_SHIFT (27)

    int ret;

    // static allocation on stack matrices (no need to free those)
    MT_MAT M_T = M_INITS(2, 2);
    MT_VEC V_Mdiag = V_INITS(2);
    MT_MAT M_product = M_INITS(2, 2);

    MT_M_AT(&M_T, 0, 0) =  1 << MAT_SHIFT;
    MT_M_AT(&M_T, 0, 1) =  1 << MAT_SHIFT;
    MT_M_AT(&M_T, 1, 0) =  1 << MAT_SHIFT;
    MT_M_AT(&M_T, 1, 1) = -1 << MAT_SHIFT;

    MT_V_AT(&V_Mdiag, 0) = 0.1*0.1 * dMSHIFT;
    MT_V_AT(&V_Mdiag, 1) = 0.01*0.01 * dMSHIFT;

//#define TIME_PERFS

#ifdef TIME_PERFS
    struct timeval time;
    gettimeofday(&time, 0);
    long start_time = 1000000 * time.tv_sec + time.tv_usec;

    for(int i = 0; i < 1000000; i++){
#endif
        ret = mt_mtdm_mlt(&M_T, &V_Mdiag, &M_product);
        assert(!ret);
#ifdef TIME_PERFS
    }

    gettimeofday(&time, 0);
    long cur_time = 1000000 * time.tv_sec + time.tv_usec;

    printf("duration: %li\n", cur_time - start_time);
#else
    printf("M_T, ");
    mt_m_output(&M_T);
    printf("V_Mdiag, ");
    mt_v_output(&V_Mdiag);
    printf("M_product, ");
    mt_m_output(&M_product);
#endif

#undef MAT_SHIFT
#undef VEC_SHIFT
}

void test_mtdm() {
#define MAT_SHIFT (27)
#define VEC_SHIFT (27)

    int ret;

    // static allocation on stack matrices (no need to free those)
    MT_MAT M_T = M_INITS(3, 2);
    MT_VEC V_Mdiag = V_INITS(3);
    MT_MAT M_product = M_INITS(2, 2);

    MT_M_AT(&M_T, 0, 0) =  1 << MAT_SHIFT;
    MT_M_AT(&M_T, 0, 1) =  1 << MAT_SHIFT;
    MT_M_AT(&M_T, 1, 0) =  1 << MAT_SHIFT;
    MT_M_AT(&M_T, 1, 1) = -1 << MAT_SHIFT;
    MT_M_AT(&M_T, 2, 0) =  2 << MAT_SHIFT;
    MT_M_AT(&M_T, 2, 1) =  3 << MAT_SHIFT;

    MT_V_AT(&V_Mdiag, 0) = 0.1*0.1 * dMSHIFT;
    MT_V_AT(&V_Mdiag, 1) = 0.01*0.01 * dMSHIFT;
    MT_V_AT(&V_Mdiag, 2) = 0.5*0.5 * dMSHIFT;

//#define TIME_PERFS

#ifdef TIME_PERFS
    struct timeval time;
    gettimeofday(&time, 0);
    long start_time = 1000000 * time.tv_sec + time.tv_usec;

    for(int i = 0; i < 1000000; i++){
#endif
        ret = mt_mtdm_mlt(&M_T, &V_Mdiag, &M_product);
        assert(!ret);
#ifdef TIME_PERFS
    }

    gettimeofday(&time, 0);
    long cur_time = 1000000 * time.tv_sec + time.tv_usec;

    printf("duration: %li\n", cur_time - start_time);
#else
    printf("M_T, ");
    mt_m_output(&M_T);
    printf("V_Mdiag, ");
    mt_v_output(&V_Mdiag);
    printf("M_product, ");
    mt_m_output(&M_product);
#endif

#undef MAT_SHIFT
#undef VEC_SHIFT
}

struct{
    void (*f)();
    char* s;
} tests[]={
        {test_machtypes,   "Machine types"},
        {test_matrixadd,   "A+B"},
        {test_linearsolve, "Ax=b solutions (using explicit matrix inversion)"},
        {test_invmatrix,   "Matrix inversion"},
        {test_mtdm_square, "Matrix^T*diag(d)*Matrix (square matrices)"},
        {test_mtdm,        "Matrix^T*diag(d)*Matrix"},
};

int main() {
    char* buf = NULL;
    for(int i = 0; i < sizeof(tests)/sizeof(*tests); i++){
        const char format[] = "# Begin of test #%02i \"%s\" #\n";

        // create border
        {
            int sz = sizeof(format) + strlen(tests[i].s) - 5;
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
