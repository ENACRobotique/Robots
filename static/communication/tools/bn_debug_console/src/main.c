/*
 * main.c
 *
 *  Created on: 10 oct. 2013
 *      Author: quentin
 */
#include <signal.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/time.h>
#include <unistd.h>
#include <malloc.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <getopt.h> // parameters parsing

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../botNet/shared/bn_utils.h"
#include "../../global_errors.h"
#include "node_cfg.h"


static int menu = 0;

void intHandler(int dummy) {
    menu = 1;
}

void usage(char *cl){
    printf("Botnet console\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--log-file, -f        output log file of received messages (overwritten)\n");
    printf("\t--add-return, -r      adds missing line return to debug strings (default)\n");
    printf("\t--no-add-return, -R   do not add missing line return to debug strings\n");
    printf("\t--verbose, -v         increases verbosity\n");
    printf("\t--quiet, -q           not verbose\n");
    printf("\t--help, -h, -?        prints this help\n");
}

int main(int argc, char **argv){
    int quit=0,quitMenu=0,err;
    char oLF=1,verbose=1;           //booleans used for display options, add new line char if missing (default yes), pure console mode (default no)
    FILE *fd = NULL;

    sMsg msgIn;
    char cmd;
    bn_Address destAd;


    // arguments parsing

        while(1){
            static struct option long_options[] = {
                    {"log-file",        required_argument,  NULL, 'f'},
                    {"add-return",      no_argument,        NULL, 'r'},
                    {"no-add-return",   no_argument,        NULL, 'R'},
                    {"verbose",         no_argument,        NULL, 'v'},
                    {"quiet",           no_argument,        NULL, 'q'},
                    {"help",            no_argument,        NULL, 'h'},
                    {NULL,              0,                  NULL, 0}
            };

            int c = getopt_long(argc, argv, "f:rRvqh?", long_options, NULL);
            if(c == -1)
                break;
            switch(c){
            case 'f':
                if(fd){
                    fclose(fd);
                }
                fd = fopen(optarg, "wb+");
                break;
            case 'r' :
                oLF=1;
                break;
            case 'R' :
                oLF=0;
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

    bn_init();

    printf("listening, CTRL+C  for menu\n");

    signal(SIGINT, intHandler);

    //main loop
    while (!quit){
        int nbTraces,f; //for traceroute display

        usleep(500);


        //receives messages, displays string if message is a debug message
        err = bn_receive(&msgIn);
        if (err < 0){
            if (err == -ERR_INTERRUPTED){
                menu=1;
            }
            else {
                fprintf(stderr, "bn_routine() error #%i\n", -err);
                if (err<=ERR_BN_INIT_FAILED) exit(1);
            }
        }
        if (err > 0){
            if (verbose>=1) {
                printf("message received from %hx, type : %s (%hhu)  ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type);
                if(fd) fprintf(fd,"message received from %hx, type : %s (%hhu)  ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type);
            }
            switch (msgIn.header.type){
            case E_DEBUG :
                printf("%s",msgIn.payload.debug);
                if(fd) fprintf(fd,"%s",msgIn.payload.debug);
                if (oLF){
                    if (!strlen((char*)msgIn.payload.debug) || (strlen((char*)msgIn.payload.debug) && msgIn.payload.debug[(unsigned int)strlen((char*)msgIn.payload.debug)-1]!='\n')) {
                        printf("\n");
                        if(fd) fprintf(fd,"\n");
                    }
                }
                break;
            case E_POS :
                printf("robot%hhu@(%fcm,%fcm,%f°)\n", msgIn.payload.pos.id, msgIn.payload.pos.x, msgIn.payload.pos.y, msgIn.payload.pos.theta*180./M_PI);
                if(fd) fprintf(fd,"message received from %hx, type : %s (%hhu)  ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type);
                break;
            default : break;
            }
        }
        else if (err < 0){
            if (err == -ERR_INTERRUPTED){
                menu=1;
            }
            else{
                fprintf(stderr, "bn_receive() error #%i\n", -err);
                exit(1);
            }
        }

        //menu
        if (menu){
            quitMenu=0;
            while (!quitMenu){
                printf("\ndebug reader menu\n");
                printf("s : send debugger address\n");
                printf("p : ping\n");
                printf("t : traceroute\n");
                printf("i : info about this node\n");
                printf("l : add/remove line return to debug strings if missing\n");
                printf("v : increase verbosity\n");
                printf("V : decrease verbosity\n");
                printf("r : return\n");
                printf("q : quit\n");

                while(isspace(cmd=getchar()));

                switch (cmd){
                case 's' :  //sends debug address to distant node
                    do{
                        printf("enter destination address\n");
                        err = scanf("%hx",&destAd);
                        if (err != 1){
                            printf("error getting destination address\n");
                        }
                    }while(err != 1);
                    if ( (err=bn_debugSendAddr(destAd)) > 0){
                        printf("signalling send\n");
                        quitMenu=1;
                    }
                    else {
                        printf("error while sending : %d\n", err);

                    }
                    break;
                case 'p' :
                    do{
                         printf("enter destination address\n");
                         err = scanf("%hx",&destAd);
                         if (err != 1){
                             printf("error getting destination address\n");
                         }
                     }while(err != 1);
                     printf("ping %hx : %d ms\n",destAd,bn_ping(destAd));
                    break;
                case 't' :
                    {
                        sTraceInfo *trInfo=NULL;
                        int depth;
                        do{
                             printf("enter destination address\n");
                             err = scanf("%hx",&destAd);
                             if (err != 1){
                                 printf("error getting destination address\n");
                             }
                        }while(err != 1);
                        do{
                             printf("enter depth\n");
                             err = scanf("%i",&depth);
                             if (err != 1 || depth <= 0){
                                 printf("error getting depth\n");
                             }
                        }while(err != 1 || depth <= 0);
                        trInfo = (sTraceInfo *)malloc(depth * sizeof(sTraceInfo));
                        nbTraces=bn_traceroute(destAd,trInfo,depth,1000);
                        for (f=0;f<nbTraces;f++){
                            printf("%hx in %d ms\n",trInfo[f].addr,trInfo[f].ping);
                        }
                    }
                    break;
                case 'i' :  //displays info about current node
                    {
                        printf("my addr (total) : %4hx\n",MYADDRX);
                        printf("my addr (local) : %4hx\n",MYADDRX&DEVICEX_MASK);
                        printf("my subnet  : %4hx\n\n",MYADDRX&SUBNET_MASK);
                    }
                    break;
                case 'l' :
                    oLF^=1;
                    if (oLF) printf("new line will be added if missing");
                    else printf("new line won't be added if missing");
                    break;
                case 'c' :
                    verbose^=1;
                    if (verbose) printf("message info won't be displayed");
                    else printf("message info will be displayed");
                    break;
                case 'r' : quitMenu=1; printf("back to listening, CTRL+C for menu\n\n"); break;
                case 'q' : quitMenu=1; quit=1; break;
                default : break;
                }
            }

            menu=0;
        }

    }

    if (fd) fclose(fd);
    printf("bye\n");

    return 0;
}
