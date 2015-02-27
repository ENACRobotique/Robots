#include <stdlib.h>
#include <stdio.h>

#include <signal.h> // ctrl+c for menu
#include <getopt.h> // parameters parsing
#include <termios.h> // terminal raw mode
#include <unistd.h> // select, read
#include <sys/time.h> // select
#include <sys/types.h> // select
#include <ctype.h> // isspace, toupper
#include <math.h> // M_PI, cos, sin
#include <errno.h> // errno

#include "millis.h" // millis()

#include "../botNet/shared/botNet_core.h"
#include "../network_tools/bn_debug.h"
#include "../network_tools/bn_intp.h"
#include "roles.h"

#include "params.h"
#include "../../global_errors.h"
#include "node_cfg.h"

#include "pos_history.h"
#include "controller.h"
#include "asserv.h"

//#define CTRLC_MENU

#ifdef CTRLC_MENU
// signals-related definitions
volatile int sigint = 0;
void sigHandler(int sig){
    sigint = 1;
}
#endif

void usage(char *cl){
    printf("Command Line Interface\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--log-file, -f        output log file (overwritten)\n");
//    printf("\t--x-initial, -x       initial x position (cm)\n");
//    printf("\t--y-initial, -y       initial y position (cm)\n");
//    printf("\t--theta-initial, -t   initial orientation theta (°)\n");
    printf("\t--verbose, -v         increases verbosity\n");
    printf("\t--quiet, -q           not verbose\n");
    printf("\t--help, -h, -?        prints this help\n");
}

void intp_sync_handler(sMsg *msg){
    bn_intp_msgHandle(msg);

    if(bn_intp_isSync()){
        tD_setLo2GlUsOffset(bn_intp_MicrosOffset);
    }
}

int main(int argc, char *argv[]){
    FILE *fd = NULL;
    sMsg inMsg = {{0}}, outMsg = {{0}};
    int ret, quit = 0;
#ifdef CTRLC_MENU
    int cmd, quitMenu = 0;
#endif
    unsigned int prevAsserv = 0, prevPos = 0, prevDbg = 0;

    // arguments parsing
    int verbose = 1;
    while(1){
        static struct option long_options[] = {
                {"log-file",        required_argument,  NULL, 'f'},
//                {"x-initial",       required_argument,  NULL, 'x'},
//                {"y-initial",       required_argument,  NULL, 'y'},
//                {"theta-initial",   required_argument,  NULL, 't'},
                {"verbose",         no_argument,        NULL, 'v'},
                {"quiet",           no_argument,        NULL, 'q'},
                {"help",            no_argument,        NULL, 'h'},
                {NULL,              0,                  NULL, 0}
        };

        int c = getopt_long(argc, argv, "f:x:y:t:vqh?", long_options, NULL);
        if(c == -1)
            break;

        switch(c){
        case 'f':
            if(fd){
                fclose(fd);
            }
            fd = fopen(optarg, "wb+");
            break;
//        case 'x':
//            x = strtof(optarg, NULL);
//            break;
//        case 'y':
//            y = strtof(optarg, NULL);
//            break;
//        case 't':
//            theta = strtof(optarg, NULL)*M_PI/180.;
//            break;
        case 'v':
            verbose++;
            break;
        case 'q':
            verbose = 0;
            break;

        default:
           printf("?? getopt returned character code 0%o ??\n", c);
           /* no break */
        case 'h':
        case '?':
            usage(argv[0]);
            exit(EXIT_FAILURE);
            break;
        }
    }

    // arguments check

    // botNet initialization
    bn_attach(E_ROLE_SETUP, role_setup);
    bn_attach(E_INTP, intp_sync_handler); // replaces bn_intp_install() to catch synchronization

    ret = bn_init();
    if(ret < 0){
        printf("bn_init() failed err#%i\n", -ret);
        exit(1);
    }

    if(verbose >= 1){
        printf("Node ready.\n");
    }

    // hardware initialization
    asserv_init();

    prevAsserv = millis();

    // main loop
    while(!quit){
        usleep(500);

        ret = bn_receive(&inMsg);
        printf("\x1b[K");
        if(ret < 0){
#ifdef CTRLC_MENU
            if(ret == -ERR_SYSERRNO && errno == EINTR){
                sigint=1;
            }
            else
#endif
            {
                fprintf(stderr, "bn_receive() error #%i\n", -ret);
                exit(1);
            }
        }
        else if(ret > 0){
            ret = role_relay(&inMsg); // relay any received message if asked to
            if(ret < 0){
                printf("role_relay() error #%i\n", ret);
            }

            switch(inMsg.header.type){
            case E_SPEED_SETPOINT:
                new_speed_setpoint(inMsg.payload.speedSetPoint.speed);
                printf("new setpoint=%.2fcm/s\n", inMsg.payload.speedSetPoint.speed);
                break;
            case E_TRAJ:
//                bn_printDbg("got traj element");

                ret = new_traj_el(&inMsg.payload.traj);
                if(ret < 0){
                    printf("new_traj_el() error #%i\n", -ret);
                }
                break;
            case E_POS:
                bn_printDbg("got pos");

                ret = new_pos(&inMsg.payload.pos);
                if(ret < 0){
                    printf("new_pos() error #%i\n", -ret);
                }
                break;
            case E_POS_QUERY:
                if(inMsg.payload.posQuery.id == ELT_PRIMARY){
                    outMsg.header.destAddr = inMsg.header.srcAddr;
                    outMsg.header.size = sizeof(outMsg.payload.genericStatus);
                    outMsg.header.type = E_GENERIC_STATUS;

                    if(!ph_get_pos(&outMsg.payload.genericStatus, tD_newFrom_GlUs(inMsg.payload.posQuery.date))){
                        bn_send(&outMsg);
                    }
                }
                break;
            default:
                bn_printDbg("got unhandled msg");
                break;
            }
        }
        printf("\x1b[spos %.2fcm, %.2fcm, %.1f°\x1b[u", I2Ds(x), I2Ds(y), RI2Rs(theta)*180./M_PI);
        fflush(stdout);

#ifdef CTRLC_MENU
        //menu
        if(sigint){
            quitMenu = 0;
            while(!quitMenu){
                printf("\n#Menu:\n");
                printf("#  r - return\n");
                printf("#  q - quit\n");

                printf("#> ");
                while(isspace(cmd=getchar()));

                switch (toupper(cmd)){
                case 'R':
                    quitMenu = 1;
                    printf("#back to listening\n");
                    break;
                case 'Q':
                    quitMenu = 1;
                    quit = 1;
                    break;
                default:
                    break;
                }
            }

            sigint=0;
        }
#endif

        if(millis() - prevAsserv >= 20){
            prevAsserv += 20;

            if(millis() - prevAsserv > 10) { // we are very late, do not take care of these data
                prevAsserv = millis();

                printf("lost asserv synchro, resyncing...\n");
                continue;
            }

            ret = new_asserv_step();
            if(ret < 0){
                printf("new_asserv_step() error #%i", -ret);
            }

            // enqueue latest position
            if(bn_intp_isSync()){
                unsigned int p_t; // local time (µs)
                sPHPos *slot = ph_get_new_slot_pointer();

                get_pos(&slot->s.prop_status.pos, &slot->s.prop_status.pos_u, &p_t);

                slot->s.date = bn_intp_micros2s(p_t); // global time
                slot->s.id = ELT_PRIMARY;

                slot->d = tD_newFrom_LoUs(p_t);

                ph_incr_new_slot_pointer();
            }
        }

        if(millis() - prevPos >= 100) {
            prevPos = millis();

            ret = send_pos();
            if(ret < 0){
                printf("send_pos() error #%i\n", -ret);
            }
        }

        if(millis() - prevDbg >= 1000){
            prevDbg = millis();

            /*ret = */show_stats();
//            if(ret < 0){
//                printf("show_stats() error #%i\n", -ret);
//            }
        }
    }

    printf("Ciao!\n");
    if(fd){
        fclose(fd);
    }
    return 0;
}