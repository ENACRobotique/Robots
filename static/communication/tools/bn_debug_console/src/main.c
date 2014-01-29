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

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../botNet/shared/bn_utils.h"
#include "../../global_errors.h"
#include "node_cfg.h"


static int menu = 0;

void intHandler(int dummy) {
    menu = 1;
}

int main(){
    int quit=0,quitMenu=0,err;
    sMsg msgIn;
    char cmd;
    bn_Address destAd;

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
                exit(1);
            }
        }
        if (err > 0){
            printf("message received from %hx, type : %s (%hhu)\n",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type);
            switch (msgIn.header.type){
            case E_DEBUG : printf("  %s\n",msgIn.payload.debug); break;
            case E_POS : printf("  robot%hhu@(%fcm,%fcm,%f°)\n", msgIn.payload.pos.id, msgIn.payload.pos.x, msgIn.payload.pos.y, msgIn.payload.pos.theta*180./M_PI); break;
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
                printf("r : return\n");
                printf("q : quit\n");

                while(isspace(cmd=getchar()));

                switch (toupper(cmd)){
                case 'S' :  //sends debug address to distant node
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
                case 'P' :
                    do{
                         printf("enter destination address\n");
                         err = scanf("%hx",&destAd);
                         if (err != 1){
                             printf("error getting destination address\n");
                         }
                     }while(err != 1);
                     printf("ping %hx : %d ms\n",destAd,bn_ping(destAd));
                    break;
                case 'T' :
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
                case 'I' :  //displays info about current node
                    {
                        printf("my addr (total) : %4hx\n",MYADDRX);
                        printf("my addr (local) : %4hx\n",MYADDRX&DEVICEX_MASK);
                        printf("my subnet  : %4hx\n\n",MYADDRX&SUBNET_MASK);
                    }
                    break;
                case 'R' : quitMenu=1; printf("back to listening, CTRL+C for menu\n\n"); break;
                case 'Q' : quitMenu=1; quit=1; break;
                default : break;
                }
            }

            menu=0;
        }

    }

    printf("bye\n");

    return 0;
}
