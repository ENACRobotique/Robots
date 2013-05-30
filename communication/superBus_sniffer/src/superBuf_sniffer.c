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

#include "messages.h"
#include "network_cfg.h"
#include "lib_Xbee_x86.h"

int bool=42;
unsigned char srcMask=0xFF, dstMask=0xFF;
char dType[E_TYPE_COUNT], rawDisplay=0;
int snetOnly=1; //display only message which destination is within our subnetwork

void exitHandler(int rien){
    char car;
    int i;
    int menuloop=1;
    do{
        printf("\nwaiting your orders:\n m: edit destination address mask (current %x)\n M: edit source address mask (current %x)\n t: add one visible type\n r: remove one visible type\n R: remove all visible types\n d: display current parameters\n s: display only message which destination is in the debugger subnetwork\n c: close this menu\n q: quit the program\n",dstMask, srcMask);
        car=getchar();
        while(getchar()!='\n');
        switch (car ){
        case 'm':
            printf("current destination mask: %x\n new mask ?\n",srcMask);
            scanf("%2x",&dstMask);
            while(getchar()!='\n');
            break;
        case 'M':
            printf("current source mask: %x\n new mask ?\n",srcMask);
            scanf("%2x",&srcMask);
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
            printf("destination msk: %2x\nsource mask: %2x\n",dstMask,srcMask);
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
    clock_t prevClock,currentClock;
    double elapsed;
    double byteRate;

    //autorizing every type of message to be displayed:
    memset(dType,1,sizeof(dType));

    if(argc!=2){
        printf("XBee sniffer & positioning system network analyser\nMade by Quentin VEY for the ENAC robotic club, 2013\n\nusage: %s ttyport (ex: /dev/ttyUSB0)\n", argv[0]);
        return(1);
     }
        
    sig_t prevH=signal(SIGINT,exitHandler);

    Xbee_initSerial(argv[1]);

    printf("ctrl+C to see the menu\n");


    while ( bool ){
        prevClock=clock();
        if ( (bytesCount=Xbee_receive(&inMsg)) ){
            if (  (!snetOnly || (inMsg.header.destAddr&SUBNET_MASK) ) && ((inMsg.header.destAddr & dstMask) || (inMsg.header.srcAddr & srcMask)) ) {
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
                            printf("%4x -> %4x DEBUG %s ,%u , %d",inMsg.header.srcAddr,inMsg.header.destAddr, inMsg.payload.debug.msg,inMsg.payload.debug.u, inMsg.payload.debug.i);
                        }
                        break;
                case E_DEBUG_ADDR:
                        if (dType[E_DEBUG_ADDR] ){
                            printf("%4x -> %4x DEBUG_ADDR %s ,%u , %d",inMsg.header.srcAddr,inMsg.header.destAddr,inMsg.payload.debug.msg,inMsg.payload.debug.u, inMsg.payload.debug.i);
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
                case E_RAW:
                        if (dType[E_RAW] ){
                            printf("%4x -> %4x RAW",inMsg.header.srcAddr,inMsg.header.destAddr);
                        }
                        break;
                default:
                        printf("%4x -> %4x , %u: unknown type ###",inMsg.header.srcAddr,inMsg.header.destAddr,inMsg.header.type);
                        break;
                }
            }
            currentClock=clock();
            elapsed = ((double) (currentClock - prevClock) * 1000) / CLOCKS_PER_SEC;
            byteRate=bytesCount/elapsed;
            printf("\033[128D\033[50C%f Ko/s\n",byteRate);
        }
    }
    if(prevH) prevH(15);
    printf(" adios\n");

    return 0;
}
