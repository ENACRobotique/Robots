/*
 * syscalls.c
 *
 *  Created on: 18 déc. 2013
 *      Author: ludo6431
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/types.h>
#include <sys/unistd.h>

#ifdef STDSTREAMS_UART0
#include "uart0.h"

static inline void __uart0_putchar(uint8_t ch){
    if(ch == '\n'){
        UARTWriteChar('\r');
    }
    else{
        UARTWriteChar(ch);
    }
}

static inline int __uart0_available(){
    return UARTReadAvailable();
}

static inline uint8_t __uart0_getchar(){
    uint8_t c = UARTReadChar();
    if(c == '\r'){
        return '\n';
    }
    else{
        return c;
    }
}
#endif

void _exit(int status){
    while (1);
}

int _close(int file){
    return -1;
}

/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env){
    errno = ENOMEM;
    return -1;
}

/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
 */
int _fork(){
    errno = EAGAIN;
    return -1;
}

/*
 fstat
 Status of an open file. For consistency with other minimal implementations in these examples,
 all files are regarded as character special devices.
 The `sys/stat.h' header file required is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st){
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */
int _getpid(){
    return 1;
}

/*
 isatty
 Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
int _isatty(int file){
    switch (file){
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}

/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig){
    errno = EINVAL;
    return (-1);
}

/*
 link
 Establish a new name for an existing file. Minimal implementation:
 */
int _link(char *old, char *new){
    errno = EMLINK;
    return -1;
}

/*
 lseek
 Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir){
    return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
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

/*
 read
 Read a character to a file. `libc' subroutines will use this system routine for input from all files, including stdin
 Returns -1 on error or blocks until the number of characters have been read.
 */
int _read(int file, char *ptr, int len){
#ifdef STDSTREAMS_UART0
    uint8_t c;
#endif
    int ret = 0;
    switch(file){
    case STDIN_FILENO:
#ifdef STDSTREAMS_UART0
        if(len <= 0)
            return 0;

        c = __uart0_getchar();
        *ptr++ = c;
        ret++;

        for(; ret < len && c != '\n'; ret++){
            if(!__uart0_available()){
                break;
            }
            c = __uart0_getchar();
            *ptr++ = c;
        }
#endif
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return ret;
}

/*
 stat
 Status of a file (by name). Minimal implementation:
 int    _EXFUN(stat,( const char *__path, struct stat *__sbuf ));
 */
int _stat(const char *filepath, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 times
 Timing information for current process. Minimal implementation:
 */
clock_t _times(struct tms *buf) {
    return -1;
}

/*
 unlink
 Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

/*
 wait
 Wait for a child process. Minimal implementation:
 */
int _wait(int *status) {
    errno = ECHILD;
    return -1;
}

/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, char *ptr, int len) {
    int n = 0;
    switch (file) {
    case STDOUT_FILENO: /*stdout*/
    case STDERR_FILENO: /* stderr */
#ifdef STDSTREAMS_UART0
        for (; n < len; n++) {
            __uart0_putchar(*ptr++);
        }
#endif
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return n;
}
