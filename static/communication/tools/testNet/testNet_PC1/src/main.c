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

#include "../../../../botNet/shared/botNet_core.h"
#include "../../../../botNet/shared/bn_utils.h"
#include "../../../../botNet/shared/bn_debug.h"
#include "../testNet_functions/bn_testFunc.h"
#include "node_cfg.h"


static int menu = 0;


void intHandler(int dummy) {
    menu = 1;
}

int main(){
    volatile int quit=0,quitMenu=0,err;
    int nbreceived=0;
    sMsg msgIn;
    char cmd;
    bn_Address destAd;
    int l;

    int sum=0;
    int rxed[64]={0};

    int msg2send=10000,msgSend=0,msgNStatused=0,msgNOk=0,avgElem=0;
    float avgMes=0,avgVal=0;
    sMsg out;
    struct timeval currentClock, prevClock;


    bn_init();
    bn_attach(E_DEBUG_SIGNALLING,&bn_debugUpdateAddr);



    printf("listening, CTRL+C  for menu\n");

    signal(SIGINT, intHandler);

    gettimeofday(&prevClock,NULL);

    while (!quit){
        usleep(500);
        bn_routine();
        memset(&msgIn,0,sizeof(sMsg));
        if (bn_receive(&msgIn)>0){
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
        cbr_deamon();


        if (menu){
            menu=0;
            quitMenu=0;
            while (!quitMenu){
                cmd=0;
                printf("nb msg received : %d\n",nbreceived);
                nbreceived=0;
                printf("\ndebug reader menu\n");
                printf("s : send debugger address\n");
                printf("i : info about this node\n");
                printf("b : send a burst of message to \n");
                printf("p : ping\n");
                printf("t : traceroute\n");
                printf("r : return\n");
                printf("q : quit\n");

                while(isspace(cmd=getchar()));

                switch (toupper(cmd)){
                case 'R' : quitMenu=1; printf("back to listening, CTRL+C for menu\n\n"); break;
                case 'Q' : quitMenu=1; quit=1; break;
                case 'S' :
                    printf("enter destination address\n");
                    scanf("%hx",&destAd);
                    if ( (err=bn_debugSendAddr(destAd)) > 0){
                        printf("signalling send\n");
                        quitMenu=1;
                    }
                    else {
                        printf("error while sending : %d\n", err);

                    }
                    break;
                case 'I' :
                    printf("my addr (total) : %4hx\n",MYADDRX);
                    printf("my addr (local) : %4hx\n",MYADDRX&DEVICEX_MASK);
                    printf("my subnet  : %4hx\n\n",MYADDRX&SUBNET_MASK);
                    break;
                case 'B' :
                    printf("enter destination address\n");
                    scanf("%hx",&out.header.destAddr);
                    out.header.type=E_TEST_PKT;

                    printf("enter number of message to send\n");
                    scanf("%d",&msg2send);

                    gettimeofday(&prevClock,NULL);
                    msgNOk=0;
                    msgNStatused=0;
                    msgSend=0;
                    while ( msg2send>=0 ){

                        out.header.size=msg2send;
                        gettimeofday(&currentClock,NULL);

                        switch (bn_sendAck(&out)){
                        case -3 : msgNOk++; break;
                        case -2 : msgNStatused++; break;
                        case -1 : msgNStatused++; break;
                        default : msgSend++; break;
                        }

                        avgElem++;
                        avgMes=(float)((currentClock.tv_sec*1000000 + currentClock.tv_usec)-(prevClock.tv_sec*1000000 + prevClock.tv_usec));
                        avgVal=avgMes/(float)avgElem +avgVal*((float)(avgElem-1))/((float)avgElem);

                        prevClock=currentClock;


                        if (msg2send==0) printf("send %d, Nstat %d, NOk %d, interval %f\n",msgSend, msgNStatused,msgNOk,avgVal);
                        msg2send--;


                    }
                    break;
                case 'P' :
                    destAd=0;
                    printf("enter destination adress\n");
                    scanf("%hx",&destAd);
                    printf("ping %hx : %d ms\n",destAd,bn_ping(destAd));
                    break;
                case 'T' :
                    {
                    destAd=0;

                    printf("enter destination adress\n");
                    scanf("%hx",&destAd);
                    int depth;
                    printf("enter depth\n");
                    scanf("%hx",&depth);
                    sTraceInfo trInfo[depth];
                    int nbTraces,f;
                    nbTraces=bn_traceroute(destAd,trInfo,depth,1000);
                    for (f=0;f<nbTraces;f++){
                        printf("%hx in %d ms\n",trInfo[f].addr,trInfo[f].ping);
                    }
                    }
                    break;
                default : break;
                }

            }

        }

    }

    printf("bye\n");
    bn_deattach(E_DEBUG_SIGNALLING);

    return 0;
}


