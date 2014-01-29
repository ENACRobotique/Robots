/*
 * timeout.h
 *
 *  Created on: 1 nov. 2013
 *      Author: quentin
 */

#ifndef TIMEOUT_H_
#define TIMEOUT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Expected behavior of testTimeout :
 *  1 - first call : start a timer of micros microsecond and return 1.
 *  2 - any call between the first call and "first call + micros"  : return 1.
 *  3 - first call after the end of the timer : return 0.
 *  next call : goto 1.
 *  testTimeout(0) MUST reset the timer : force next call to be in state 1
 *
 *
 *  store is a pointer to a storing value, to enable nesting
 *
 *  /!\ watch out "store" for nesting /!\
 */
int testTimeout(uint32_t delay, uint32_t *store);

/* stopwatch : a simple stopwatch utility.
 * Arguments :
 *  store : pointer to a memory area. MUST be set to 0 in order to "start" the stopwatch.
 *          After the first call, its value is undefined but MUST NOT be modified until the user wants to stop measuring time.
 * Return value :
 *  number of microsecond elapsed since the first call of stopwatch() after *store was set to 0
 * Expected behavior of stopwatch :
 *   1 - *store is set to 0 prior to first call (resets the stopwatch)
 *   2 - first call after that : returns 0 and starts the stopwatch.
 *   3 - any call after that (and before any modification to *store) : returns the amount of time elapsed after step 2.
 *   4 - *store is modified (not mandatory). If *store is set to 0, goto 1.
 *
 *  store is a pointer to a storing value, to enable nesting
 *
 *  /!\ watch out "store" for nesting /!\
 *  store must me initialized at 0
 *  and re-set to 0 before re-use
 */
uint32_t stopwatch(uint32_t *store);

#ifdef __cplusplus
}
#endif

#endif /* TIMEOUT_H_ */
