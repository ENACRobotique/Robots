/*
 * syscall.c
 *
 *  Created on: 16 mai 2014
 *      Author: quentin
 */

#include <sys/types.h>

/**
 * _sbrk - newlib memory allocation routine
 */
extern char _end_bss;

caddr_t _sbrk (int incr)
{
    double current_sp;
    static char * heap_end = NULL;
    char * prev_heap_end;

    if (heap_end == NULL) {
        heap_end = &_end_bss; /* first ram address after bss and data */
    }

    prev_heap_end = heap_end;

    // simplistic approach to prevent the heap from corrupting the stack
    // TBD: review for alternatives
    if ( heap_end + incr < (caddr_t)&current_sp ) {
        heap_end += incr;
        return (caddr_t) prev_heap_end;
    }
    else {
        return NULL;
    }
}
