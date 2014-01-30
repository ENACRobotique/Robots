/*
 * main.c
 *
 *  Created on: 16 janv. 2014
 *      Author: quentin
 */

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
#include <string.h>
#include <stdlib.h>

#include "../../../../botNet/shared/botNet_core.h"
#include "../../../../botNet/shared/bn_utils.h"
#include "../../../../botNet/shared/bn_debug.h"
#include "../testNet_functions/bn_testFunc.h"
#include "../../../../global_errors.h"
#include "node_cfg.h"


#define DEBUG


static int menu = 0;


void intHandler(int dummy) {
    menu = 1;
}

int main(){
    sMsg msgIn;

    bn_Address destAd;
    int ret,quitMenu,nbTraces,quit=0,f;
    char cmd;
    int err;

    bn_init();
    bn_attach(E_DEBUG_SIGNALLING,&bn_debugUpdateAddr);


    printf("init terminé, mon adresse est : %hx\n",MYADDRU);

    signal(SIGINT, intHandler);
    printf("listening, CTRL+C  for menu\n");



    while (!quit){
        usleep(500);

        memset(&msgIn,0,sizeof(sMsg));

        //message reception
        if ( (ret=bn_receive(&msgIn))>0 ){
            // some display and message handling
            printf("message received from %hx, type : %u\n",msgIn.header.srcAddr,msgIn.header.type);
            switch (msgIn.header.type){
            case E_DEBUG : printf("%s\n",msgIn.payload.debug); break;
            case E_ACK_RESPONSE : printf("ack response : %d, ack addr %hx\n",msgIn.payload.ack.ans,msgIn.payload.ack.addr); break;
            case E_WELL_CTRL :
            case E_TEST_PKT :
                well_deamon(&msgIn);
                break;
            case E_CBR_CTRL :
                cbr_controller(&msgIn);
                break;
            default :
                break;
            }
        }
        else if (ret < 0){
            if (ret == -ERR_INTERRUPTED){
                menu=1;
            }
            else if(ret != -ERR_UART_READ_BYTE_TIMEOUT){
                fprintf(stderr, "bn_routine() error #%i\n", -ret);
                exit(1);
            }
        }

        //menu (some control functions, like bn_debug_console)
        if (menu){
            quitMenu=0;
            while (!quitMenu){
                printf("\ndebug reader menu\n");
                printf("s : send debugger address\n");
                printf("p : ping\n");
                printf("t : traceroute\n");
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
                case 'R' : quitMenu=1; printf("back to listening, CTRL+C for menu\n\n"); break;
                case 'Q' : quitMenu=1; quit=1; break;
                default : break;
                }
            }

            menu=0;
        }

    }

    printf("bye\n");
    bn_deattach(E_DEBUG_SIGNALLING);

    return 0;
}


