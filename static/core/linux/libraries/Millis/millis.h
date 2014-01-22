#ifndef _MILLIS_H
#define _MILLIS_H

/* millis :
 * Argument : none
 * Return value : the amout of millisecond elapsed since an arbitrary and undefined moment.
 *
 * If millis() returns x at time t, a call of millis at time t+n (n in ms) returns x+n (expect if buffer overflow)
 * May (and will certainly) overflow
 * Based on gettimeofday from  <sys/time.h>
 */
int millis();

/* micros :
 * Argument : none
 * Return value : the amout of microsecond elapsed since an arbitrary and undefined moment.
 *
 * If micros() returns x at time t, a call of micros at time t+n (n in µs) returns x+n (expect if buffer overflow)
 * May (and will certainly) overflow
 * Based on gettimeofday from  <sys/time.h>
 */
int micros();


#endif
