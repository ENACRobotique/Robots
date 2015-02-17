#include <mt_io.h>
#include <mt_mat.h>
#include <mt_vec.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void test_linearsolve(){
    // containers initialization, via function call:  (internal call to malloc(), be sure to call mt_*_free() when done)
    MT_MAT A;       mt_m_init(&A, 2, 2);
    MT_MAT Am1;     mt_m_init(&Am1, 2, 2);
    MT_VEC b;       mt_v_init(&b, 2);
    // or via constant initializer on stack:  (you don't need to call mt_*_free() on those objects but the memory where it points will be released at the end of this function)
    MT_MAT AAm1   = MT_M_INITS(2, 2);
    MT_VEC x      = MT_V_INITS(2);
    MT_VEC res    = MT_V_INITS(2);

    printf("b  : %p\n", b.ve);
    printf("x  : %p\n", x.ve);
    printf("res: %p\n", res.ve);

    printf("sizeof(int)=%lu\n", sizeof(int));
    printf("sizeof(long int)=%lu\n", sizeof(long int));
    printf("sizeof(long long int)=%lu\n", sizeof(long long));
    printf("sizeof(int*)=%lu\n", sizeof(int*));
    printf("sizeof(MT_VEC)=%lu\n", sizeof(MT_VEC));

    // problem initialization
    MT_M_AT(&A, 0, 0) = 1<<MT_MAT_SHIFT;
    MT_M_AT(&A, 0, 1) = 2<<MT_MAT_SHIFT;
    MT_M_AT(&A, 1, 0) = 3<<MT_MAT_SHIFT;
    MT_M_AT(&A, 1, 1) = 4<<MT_MAT_SHIFT;

    // inversion of A
    mt_m_inv(&A, &Am1);

    // computes A*A^-1 to verify we get the identity
    mt_mm_mlt(&A, &Am1, &AAm1);

    printf("A:\n");
    mt_m_output(&A);
    printf("A^-1:\n");
    mt_m_output(&Am1);
    printf("A*A^-1:\n");
    mt_m_output(&AAm1);

    b.ve[0] = 1<<MT_VEC_SHIFT;
    b.ve[1] = -2<<MT_VEC_SHIFT;

    // calculation of A^-1 * b
    mt_mv_mlt(&Am1, &b, &x);

    printf("b:\n");
    mt_v_output(&b);
    printf("x:\n");
    mt_v_output(&x);

    // verification (calculates b - A*x)
    mt_mv_mltadd(&b, -1<<MT_VEC_SHIFT, &A, &x, &res);

    printf("residu:\n");
    mt_v_output(&res);

    // free allocations
    mt_m_free(&A);
    mt_m_free(&Am1);
    mt_v_free(&b);
    // calls to mt_*_free() do nothing in case of stack-allocated object, you can omit those
    mt_m_free(&AAm1);
    mt_v_free(&x);
    mt_v_free(&res);
}

void test_invmatrix() {
	#define dMSHIFT ((double)(1 << MT_MAT_SHIFT))
	const int32_t mat_rob2pods[3][3] = {
	    {-0.5 * dMSHIFT,  0.866025403784439 * dMSHIFT, 15.5 * dMSHIFT},
	    {-0.5 * dMSHIFT, -0.866025403784439 * dMSHIFT, 15.5 * dMSHIFT},
	    { 1.  * dMSHIFT,  0                 * dMSHIFT, 15.5 * dMSHIFT}
	};

	// static allocation on stack matrices (no need to free those)
	MT_MAT M_rob2pods = MT_M_INITS(3, 3);
	MT_MAT M_pods2rob = MT_M_INITS(3, 3);

	// init input matrix
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			MT_M_AT(&M_rob2pods, i, j) = mat_rob2pods[i][j];
		}
	}

	mt_m_inv(&M_rob2pods, &M_pods2rob);

    printf("M_rob2pods, ");
    mt_m_output(&M_rob2pods);
    printf("M_pods2rob, ");
    mt_m_output(&M_pods2rob);
}

struct{
    void (*f)();
    char* s;
} tests[]={
        {test_linearsolve, "Ax=b solutions (using explicit matrix inversion)"},
        {test_invmatrix,   "Matrix inversion"},
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
