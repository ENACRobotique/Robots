/*
 * main_tests.c
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#include <stdio.h>
#include <tests/pos_history-test.h>
#include <tests/pos_uncertainty-test.h>

typedef void (*ptestf)();
struct{
    ptestf f;   // test's function pointer
    char *n;    // test name
} tests[] = {
        {testmain_pos_history, "pos_history"},
        {testmain_pos_uncertainty, "pos_uncertainty"},
};

#define NB_TESTS (sizeof(tests) / sizeof(*tests))

int main(int argc, char *argv[]){
    int i;
    for(i = 0; i < NB_TESTS; i++){
        printf("################################################\n");
        printf("## STARTING TEST: %s\n", tests[i].n);
        printf("################################################\n");

        tests[i].f();

        printf("################################################\n");
        printf("## END OF TEST: %s\n", tests[i].n);
        printf("################################################\n");
    }

    return 0;
}
