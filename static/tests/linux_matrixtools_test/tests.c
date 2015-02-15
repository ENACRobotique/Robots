#include <mt_io.h>
#include <mt_mat.h>
#include <mt_vec.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void test_linearsolve(){
    MT_MAT A;
    MT_MAT Am1;
    MT_MAT AAm1;
    MT_VEC b;
    MT_VEC x;
    MT_VEC res;

    // containers initializes
    mt_m_init(&A, 2, 2);
    mt_m_init(&Am1, 2, 2);
    mt_m_init(&AAm1, 2, 2);
    mt_v_init(&b, 2);
    mt_v_init(&x, 2);
    mt_v_init(&res, 2);

    // problem initialization
    MT_M_AT(&A, 0, 0) = 1<<MT_MAT_SHIFT;
    MT_M_AT(&A, 0, 1) = 2<<MT_MAT_SHIFT;
    MT_M_AT(&A, 1, 0) = 3<<MT_MAT_SHIFT;
    MT_M_AT(&A, 1, 1) = 4<<MT_MAT_SHIFT;

    // inversion of A
    mt_m_inv(&A, &Am1);
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
    mt_m_free(&AAm1);
    mt_v_free(&b);
    mt_v_free(&x);
    mt_v_free(&res);
}

struct{
    void (*f)();
    char* s;
} tests[]={
        {test_linearsolve, "Ax=b solutions (using explicit matrix inversion"},
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
