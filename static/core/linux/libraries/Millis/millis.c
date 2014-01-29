
#include "millis.h"
#include "sys/time.h"
#include <stdlib.h>

/* millis :
 * Argument : none
 * Return value : the amout of millisecond elapsed since an arbitrary moment (may be the epoch if it we have big enough int, 64 bits should fit).
 *
 * If millis() returns x at time t, a call of millis at time t+n (n in ms) returns x+n (expect if buffer overflow)
 * May (and will certainly) overflow
 * Based on gettimeofday from  <sys/time.h>
 */
int millis(){
    struct timeval clock;
    gettimeofday(&clock,NULL);
    return clock.tv_usec + clock.tv_sec*1000000;
}

/* micros :
 * Argument : none
 * Return value : the amout of microsecond elapsed since an arbitrary moment (may be the epoch if it we have big enough int, 64 bits should fit).
 *
 * If micros() returns x at time t, a call of micros at time t+n (n in µs) returns x+n (expect if buffer overflow)
 * May (and will certainly) overflow
 * Based on gettimeofday from  <sys/time.h>
 */
int micros(){
    struct timeval clock;
    gettimeofday(&clock,NULL);
    return clock.tv_usec + clock.tv_sec*1000000;
}
