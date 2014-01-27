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
#include <math.h>

#include "../../../../botNet/shared/botNet_core.h"
#include "../../../../botNet/shared/bn_utils.h"
#include "../../../../botNet/shared/bn_debug.h"
#include "../../../../../global_errors.h"
#include "../testNet_functions/bn_testFunc.h"
#include "node_cfg.h"


#include "../../../../UART_framing/shared/lib_UART_framing.h" //for test purposes
#include "../../../../UART_framing/linux/lib_UART_framing_linux.h"


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
    int ret;
    float var=0;
    int i;

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
        if ( (ret=bn_routine())<0 ) printf("bn_routine error : %d\n",ret);
        memset(&msgIn,0,sizeof(sMsg));
        if (bn_receive(&msgIn)>0){
            printf("message received from %hx, type : %u\n",msgIn.header.srcAddr,msgIn.header.type);
            switch (msgIn.header.type){
            case E_DEBUG : printf("%s\n",msgIn.payload.debug); break;
            case E_DATA : //plot raw payload
                for ( i=0 ; i<msgIn.header.size && i<54; i++){
                    printf(" %hx",msgIn.payload.raw[i]);
                }
                printf("\n");
                break;
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
                    avgMes=0;
                    avgVal=0;
                    avgElem=0;
                    var=0;

                    printf("enter destination address\n");
                    scanf("%hx",&out.header.destAddr);
                    out.header.type=E_TEST_PKT;

                    printf("enter number of message to send\n");
                    scanf("%d",&msg2send);

                    gettimeofday(&prevClock,NULL);
                    msgNOk=0;
                    msgNStatused=0;
                    msgSend=0;
                    while ( msg2send>0 ){

                        out.header.size=1;

                        switch (ret=bn_sendAck(&out)){
                        case 1 : msgSend++; break;
                        case -ERR_BN_ACK_TIMEOUT : msgNStatused++; break;
                        case -ERR_BN_NACK_BROKEN_LINK : msgNOk++; break;
//                        default : printf("blah %d %d\n",ret,msg2send);break;
                        }

                        gettimeofday(&currentClock,NULL);
                        if (ret>=0){
                            avgElem++;
                            avgMes=(float)((currentClock.tv_sec*1000000 + currentClock.tv_usec)-(prevClock.tv_sec*1000000 + prevClock.tv_usec));
                            avgVal=avgMes/(float)avgElem +avgVal*((float)(avgElem-1))/((float)avgElem);

                            if (avgElem>1) var=((avgElem-1)*var)/avgElem + (avgMes-avgVal)*(avgMes-avgVal)/(avgElem-1);

                            fprintf(stderr,"%f,%f,%f\n", avgMes, avgVal, var);
                        }

                        prevClock=currentClock;


                        if (msg2send==1) printf("send(acked) %d, timeout %d, Nack bkn link %d, interval %f , stdev %f \n",msgSend, msgNStatused,msgNOk,avgVal,sqrt(var));
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
                    scanf("%i",&depth);
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


