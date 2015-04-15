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

#include "../botNet/shared/botNet_core.h"
#include "../network_tools/bn_debug.h"
#include "global_errors.h"
#include "node_cfg.h"
#include "roles.h"

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
    printf("\t--verbose, -v         increases verbosity\n");
    printf("\t--quiet, -q           not verbose\n");
    printf("\t--help, -h, -?        prints this help\n");
}

int main(int argc, char *argv[]){
    FILE *fd = NULL;
    sMsg inMsg;
    int ret, quit = 0;
#ifdef CTRLC_MENU
    int cmd, quitMenu = 0;
#endif

    // arguments parsing
    int verbose = 1;
    while(1){
        static struct option long_options[] = {
                {"log-file",        required_argument,  NULL, 'f'},
                {"verbose",         no_argument,        NULL, 'v'},
                {"quiet",           no_argument,        NULL, 'q'},
                {"help",            no_argument,        NULL, 'h'},
                {NULL,              0,                  NULL, 0}
        };

        int c = getopt_long(argc, argv, "f:vqh?", long_options, NULL);
        if(c == -1)
            break;

        switch(c){
        case 'f':
            if(fd){
                fclose(fd);
            }
            fd = fopen(optarg, "wb+");
            break;
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
    ret = bn_init();
    if(ret < 0){
        printf("bn_init() failed err#%i\n", -ret);
        exit(1);
    }

    if(verbose >= 1){
        printf("Node ready.\n");
    }

    // main loop
    while(!quit){
        ret = bn_receive(&inMsg);
        if(ret < 0){
#ifdef CTRLC_MENU
            if(ret == -ERR_SYSERRNO && errno == EINTR){
                sigint=1;
            }
            else
#endif
            {
                fprintf(stderr, "bn_receive() error #%i\n", -ret);
                if(ret == -ERR_SYSERRNO){
                    fprintf(stderr, "errno=%i\n", errno);
                }
//                exit(1);
            }
        }

        if(ret > 0 && fd){
            fprintf(fd, "message received from %03hx, type : %s (%hhu)\n", inMsg.header.srcAddr, eType2str(inMsg.header.type), inMsg.header.type);
        }
        if(ret > 0 && verbose >= 1){
            printf("message received from %03hx, type : %s (%hhu)\n", inMsg.header.srcAddr, eType2str(inMsg.header.type), inMsg.header.type);
        }

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
    }

    printf("Ciao!\n");
    if(fd){
        fclose(fd);
    }
    return 0;
}
