/*
 * syscalls.c
 *
 *  Created on: 18 déc. 2013
 *      Author: ludo6431
 */

#include <sys/types.h>
#include <errno.h>

extern unsigned char _bss_end;

caddr_t _sbrk(int incr){
    static unsigned char* current_heap_end = NULL;
    unsigned char* current_block_address;
    register unsigned char * stack_ptr asm ("sp");

    if(current_heap_end == NULL){
        current_heap_end = &_bss_end;
    }

    current_block_address = current_heap_end;

    incr = (incr + 3) & (~3); // align value to 4
    if(current_heap_end + incr > stack_ptr - 256 /* some security */){ // heap has overflowed
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    current_heap_end += incr;

    return (caddr_t)current_block_address;
}
