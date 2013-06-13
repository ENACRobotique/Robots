#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include "messages.h"
#include "lib_superBus.h"
#include "network_cfg.h"
#include "lib_Xbee_x86.h"

sig_t prevH = NULL;

void sigHandler(int sig) {
    printf("bye bye\n");
    Xbee_deInitSerial(); // FIXME when available: call sb_deinit() instead
    if(prevH) {
        prevH(sig);
    }
    exit(0);
}

int main(/*int argc, char *argv[]*/) {
    sMsg inMsg;
    int bytesCount;
//    struct timeval prevClock, currentClock;
//    float elapsed;
//    float byteRate;

    printf("Debug console\n");

    prevH = signal(SIGINT, sigHandler);

    sb_init();

    printf("ctrl+C to see the menu\n");

    while(1) {
        sb_routine();

        if((bytesCount = sb_receive(&inMsg)) > 0) {
//            gettimeofday(&prevClock, NULL);

            printf("received %i bytes from %04x: type %s\n", bytesCount, inMsg.header.srcAddr, eType2str(inMsg.header.type));

            if(inMsg.header.type == E_PERIOD) {
                printf("  %u\n", inMsg.payload.period);
            }
            else if(inMsg.header.type == E_DEBUG) {
                printf("  %s,%i,%u\n", inMsg.payload.debug.msg, inMsg.payload.debug.i, inMsg.payload.debug.u);
            }

//            gettimeofday(&currentClock, NULL);
//            elapsed = (float)(currentClock.tv_sec - prevClock.tv_sec) + ((float) (currentClock.tv_usec - prevClock.tv_usec))/1000000.;
//            printf("\033[128D\033[50C%f o/s\n", (float)bytesCount/elapsed);
        }
    }

    // never reached
    return 0;
}
