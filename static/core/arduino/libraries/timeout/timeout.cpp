/*
 * timeout.cpp
 *
 *  Created on: 1 nov. 2013
 *      Author: quentin
 */

#include "timeout.h"
#include "Arduino.h"


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
int testTimeout(uint32_t delay, uint32_t *store) {
    if (delay == 0) {
        *store = 0;
        return 0;
    }

    if (!(*store)) {
        *store = micros();
        return 1;
    }
    if (*store) {
        if ((micros() - *store) >= delay) {
            *store=0;
            return 0;
        }
        return 1;
    }
    return 0;

}
