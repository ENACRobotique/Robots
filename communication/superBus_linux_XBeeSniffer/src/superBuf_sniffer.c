
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

#include "messages.h"
#include "network_cfg.h"
#include "lib_Xbee_x86.h"

unsigned char srcMask=0xFF, dstMask=0xFF;
char dType[E_TYPE_COUNT], rawDisplay=0;
int snetOnly=1; //display only message which destination is within our subnetwork
sig_t prevH = NULL;

void sigHandler(int sig){
    char car;
    int i;
    int menuloop=1;
    do{
        printf("\nwaiting your orders:\n"
        		" m: edit destination address mask (current %x)\n"
        		" M: edit source address mask (current %x)\n"
        		" t: add one visible type\n"
        		" r: remove one visible type\n"
        		" R: remove all visible types\n"
        		" d: display current parameters\n"
        		" s: display only message which destination is in the debugger subnetwork\n"
        		" c: close this menu\n"
        		" q: quit the program\n",dstMask, srcMask);
        car=getchar();
        while(getchar()!='\n');
        switch(car){
        case 'm':
            printf("current destination mask: %x\n new mask ?\n",srcMask);
            scanf("%2hhx",&dstMask);
            while(getchar()!='\n');
            break;
        case 'M':
            printf("current source mask: %x\n new mask ?\n",srcMask);
            scanf("%2hhx",&srcMask);
            while(getchar()!='\n');
            break;
        case 't':
            printf("type (displayed):\n");
            for (i=0;i<E_TYPE_COUNT;i++)  printf("%d %s   (%s)\n",i,eType2str(i),  (dType[i]>0?"yes":"no") );
            printf("enter the *NUMBER* of the type you want to see\n");
            scanf("%d",&i);
            while(getchar()!='\n');
            if (i<0 || i>E_TYPE_COUNT){
                printf("wrong number\n");
                break;
            }
            dType[i]=1;
            break;
        case 'r':
            printf("type (displayed):\n");
            for (i=0;i<E_TYPE_COUNT;i++)  printf("%d %s   (%s)\n",i,eType2str(i),  ((dType[i]>0)?("yes"):("no")) );
            printf("enter the *NUMBER* of the type you don't want to see\n");
            scanf("%d",&i);
            while(getchar()!='\n');
            if (i<0 || i>E_TYPE_COUNT){
                printf("wrong number\n");
                break;
            }
            dType[i]=0;
            break;
        case 'R':
            printf("type displayed: none \n");
            memset(dType,0,sizeof(dType));
            break;
        case 's':
            snetOnly^=1;
            printf("subnet only : %s\n\tRemark : only messages transmitted within the Xbee network of this debugger are visible, even if \"subnet only\" is disabled\n\npress return",((snetOnly>0)?("enabled"):("disabled")) );
            getchar();
            break;
        case 'd':
            printf("destination mask: %2x\nsource mask: %2x\n",dstMask,srcMask);
            printf("type (displayed):\n");
            for (i=0;i<E_TYPE_COUNT;i++)  printf("%d %s   (%s)\n",i,eType2str(i),  ((dType[i]>0)?("yes"):("no")) );
            printf("press return\n");
            getchar();
            break;
        case 'c':
            menuloop=0;
            break;
        case 'q':
            printf("bye bye\n");
            Xbee_deInitSerial();
            if(prevH){
            	prevH(sig);
            }
            exit(0);
            break;
        default:
            printf("wrong | no choice\n");
            break;
        }
    } while (menuloop);
    printf("back to sniffing\n\n\n");
    return;
}

int main(int argc, char *argv[]){
    sMsg inMsg;
    double bytesCount;
    struct timeval prevClock,currentClock;
    double elapsed;
    double byteRate;

    // authorizing every type of message to be displayed:
    memset(dType,1,sizeof(dType));

    if(argc!=2){
        printf("XBee sniffer & superBus network analyzer\nMade by Quentin VEY for the ENAC robotic club, 2013\n\nusage: %s ttyport (ex: /dev/ttyUSB0)\n", argv[0]);
        return(1);
    }

    prevH=signal(SIGINT,sigHandler);

    Xbee_initSerial(argv[1]);

    printf("ctrl+C to see the menu\n");

    while ( 1 ){
        gettimeofday(&prevClock, NULL);
        if ( (bytesCount=Xbee_receive(&inMsg)) ){
            if ( (!snetOnly || (inMsg.header.destAddr&SUBNET_MASK) ) && ((inMsg.header.destAddr & dstMask) || (inMsg.header.srcAddr & srcMask)) ) {
                switch (inMsg.header.type){
                case E_SWITCH_CHANNEL:
                        if (dType[E_PERIOD] ){
                            printf("%4x -> %4x CHANNEL %x",inMsg.header.srcAddr, inMsg.header.destAddr , inMsg.payload.channel);
                        }
                        break;
                case E_PERIOD:
                        if (dType[E_PERIOD] ){
                            printf("%4x -> %4x PERIOD %u us",inMsg.header.srcAddr,inMsg.header.destAddr, inMsg.payload.period);
                        }
                        break;
                case E_DEBUG:
                        if (dType[E_DEBUG] ){
                            printf("%4x -> %4x DEBUG %s ,%d , %u",inMsg.header.srcAddr,inMsg.header.destAddr, inMsg.payload.debug.msg,inMsg.payload.debug.i, inMsg.payload.debug.u);
                        }
                        break;
                case E_MEASURE:
                        if (dType[E_MEASURE] ){
                            printf("%4x -> %4x MEASURE: %u at %u",inMsg.header.srcAddr,inMsg.header.destAddr, inMsg.payload.measure.value, inMsg.payload.measure.date);
                        }
                        break;
                case E_SYNC_OK:
                        if (dType[E_SYNC_OK]){
                            printf("%4x -> %4x syncOK",inMsg.header.srcAddr,inMsg.header.destAddr);
                        }
                        break;
                case E_SYNC_EXPECTED_TIME:
                        if (dType[E_SYNC_EXPECTED_TIME] ){
                            printf("%4x -> %4x sync_expected_time:%u",inMsg.header.srcAddr,inMsg.header.destAddr,inMsg.payload.syncTime);
                        }
                        break;
                case E_DATA:
                        if (dType[E_DATA] ){
                            printf("%4x -> %4x RAW",inMsg.header.srcAddr,inMsg.header.destAddr);
                        }
                        break;
                default:
                        printf("%4x -> %4x , %u: unknown type ###",inMsg.header.srcAddr,inMsg.header.destAddr,inMsg.header.type);
                        break;
                }
            }
            gettimeofday(&currentClock, NULL);
            elapsed = currentClock.tv_sec - prevClock.tv_sec + ((double) (currentClock.tv_usec - prevClock.tv_usec) )/ 1000000.;
            byteRate=bytesCount/elapsed;
            printf("\033[128D\033[60C| %f o/s\n",byteRate);
        }
    }
    // never reached
    return 0;
}
