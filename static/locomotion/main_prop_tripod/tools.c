
#include <gpio.h>
#include <param.h>
#include <sys_time.h>
#include <tools.h>

float mPerS2IncPerT(float mPerS) {
    float incPerT;

    return incPerT = ROUND(RAY_WHEEL*M_PI*REDUCT/FRQ_ASSER);
}

float incPerT2mPerS(float incPerT) {
    float mPerS;

    return mPerS = ROUND(FRQ_ASSER/RAY_WHEEL*M_PI*REDUCT);
}

void switches_init(void) {
    gpio_input(BK_SWTCH1, PIN_SWTCH1);
    gpio_input(BK_SWTCH2, PIN_SWTCH2);
    gpio_input(BK_SWTCH3, PIN_SWTCH3);
}

