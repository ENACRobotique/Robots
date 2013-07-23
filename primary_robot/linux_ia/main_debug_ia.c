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
#include <math.h>
#include <unistd.h>

#include "params.h"
#include "messages.h"
#include "lib_superBus.h"
#include "network_cfg.h"
#include "lib_Xbee_x86.h"


sig_t prevH = NULL;
enum {
    S_IDLE,
    S_TESTTRAJ,
} eState = S_IDLE;
int send_pos = 1;

uint8_t traj_extract_idx = 0;
sTrajElRaw_t traj_blue[] = {
  {7.50, 100.00, 88.75, 112.90, 82.26, 90.00, 105.00, 8.00, 16.43, 0, 0},
  {97.58, 102.44, 82.42, 57.56, 47.37, 90.00, 55.00, 8.00, 38.89, 0, 1},
  {93.66, 62.11, 20.00, 100.00, 82.83, 20.00, 100.00, 0.00, 0.00, 0, 2}
};

void sigHandler(int sig) {
    char l[16], *p;

    printf("\nmenu:\n");
    printf(" t: send test trajectory (start pos (%.2f,%.2f,%.2f°))\n", traj_blue[0].p1_x, traj_blue[0].p1_y, 0.*180./M_PI); // TODO
    printf(" q: quit the program\n");
    p = fgets(l, sizeof(l), stdin);
    if(!p)
        return;
    while(isspace(*p)) p++; // do not care about leading spaces
    switch(*p){
    case 't':
//        if( traj_extract_idx != 0 ) {
            send_pos = 1;
//        }
        traj_extract_idx = 0;
        eState = S_TESTTRAJ;
        break;
    case 'q':
        printf("bye bye\n");
        Xbee_deInitSerial(); // FIXME when available: call sb_deinit() instead
        if(prevH){
            prevH(sig);
        }
        exit(0);
        break;
    default:
        break;
    }
}

#define TV_DIFF_S(p, c) ( (float)((c)->tv_sec - (p)->tv_sec) + ((float) ((c)->tv_usec - (p)->tv_usec))/1000000. )

int main(/*int argc, char *argv[]*/) {
    sMsg msg;
    int bytesCount;
    int ret;
    int etat_beacon = 0;
    struct timeval prevClock, prevClock2, currentClock;
    int nb_perte = 0;

    printf("Debug console\n");

    prevH = signal(SIGINT,sigHandler);

    sb_init();

    printf("ctrl+C to see the menu\n");

    gettimeofday(&prevClock, NULL);
    gettimeofday(&prevClock2, NULL);
    while(1) {
        sb_routine();

        if((bytesCount = sb_receive(&msg)) > 0 && msg.header.type != E_PERIOD) {
            if(msg.header.type != E_DATA)
            printf("received %i bytes from %04x: type %s\n", bytesCount, msg.header.srcAddr, eType2str(msg.header.type));

            switch(msg.header.type) {
            case E_DEBUG:
                printf("  %s,%i,%u\n", msg.payload.debug.msg, msg.payload.debug.i, msg.payload.debug.u);
                break;
            case E_DATA:
//                printf("  %s\n", msg.payload.raw);

                nb_perte--;

                printf("nb_perte=%i\n", nb_perte);
                break;
            case E_POS:
                printf("  robot id%i @ (%.2fcm, %.2fcm, %.2f°)\n", msg.payload.pos.id, msg.payload.pos.x, msg.payload.pos.y, msg.payload.pos.theta);
                break;
            default:
                break;
            }
        }

        switch(eState) {
        default:
        case S_IDLE:
            gettimeofday(&currentClock, NULL);
            if(TV_DIFF_S(&prevClock, &currentClock) > 0.02) {
                prevClock = currentClock;

                msg.header.destAddr = ADDRI_MAIN_PROP;
                msg.header.type = E_DATA;
                msg.header.size = 40;
                msg.payload.raw[0] = (etat_beacon = !etat_beacon);
                ret = sb_send(&msg);

                if(ret >0)
                    nb_perte++;
            }
            break;
        case S_TESTTRAJ:
            gettimeofday(&currentClock, NULL);
            if(TV_DIFF_S(&prevClock2, &currentClock) > 0.02) {
                prevClock2 = currentClock;

                if(send_pos) {
                    send_pos = 0;

                    msg.header.destAddr = ADDRI_MAIN_PROP;
                    msg.header.type = E_POS;
                    msg.header.size = sizeof(sPosPayload);
                    msg.payload.pos.x = traj_blue[0].p1_x;
                    msg.payload.pos.y = traj_blue[0].p1_y;
                    msg.payload.pos.theta = 0.;
                    msg.payload.pos.id = 0; // primaire
                    ret = sb_send(&msg);

                    printf("sent%i pos\n", ret);
                }
                else if(traj_extract_idx < sizeof(traj_blue)/sizeof(*traj_blue)) {
                    msg.header.destAddr = ADDRI_MAIN_PROP;
                    msg.header.type = E_TRAJ;
                    msg.header.size = sizeof(sTrajElRaw_t);
                    memcpy(&msg.payload.traj, &traj_blue[traj_extract_idx], sizeof(sTrajElRaw_t));
                    ret = sb_send(&msg);

                    printf("sent%i traj%hhu\n", ret, traj_extract_idx);

                    traj_extract_idx++;
                }
            }
            break;
        }
    }

    // never reached
    return 0;
}

