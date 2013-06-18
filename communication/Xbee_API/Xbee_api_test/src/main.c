/*
 * main.c
 *
 *  Created on: 14 juin 2013
 *      Author: quentin
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "params.h"
#include "Xbee_API.h"

int main(int argc, char *argv[]){
    spAPISpecificStruct struIn,struOut;
    struct timeval prevClock,currentClock;
    int send=0,statused=0,acked=0,diff=0;
    int readB;
    char str[32];

    serialInit(0,argv[1]);
    printf("waiting for reset\n");
    //wait until the first frame is received (modem satus 0 : hardware reset)
    do{
        XbeeReadFrame(&struIn);
    }while(struIn.APID!=0x8a || struIn.data.modemStatus!=XBEE_MODEM_S_HARDRST);

    printf("Xbee started\n");
    XbeeATCmd("MY", 42, XBEE_ATCMD_SET, MYADDRI);
    printf("address set send\n");

    //waits for command acknowledgement
    do {
        XbeeReadFrame(&struIn);
    }while (struIn.APID!=XBEE_APID_ATRESPONSE || struIn.data.ATResponse.frameID!=42);
    printf("address set acked\n");

    //writes config
    XbeeATCmd("WR", 32, XBEE_ATCMD_SET, 0);
    //waits for command acknowledgement
        do {
            XbeeReadFrame(&struIn);
        }while (struIn.APID!=XBEE_APID_ATRESPONSE || struIn.data.ATResponse.frameID!=32);
        printf("Write acked\n");


    sprintf(str,"ping");
    // send first ping
//    XbeeTx16(0x1234,0,0x88,str,strlen(str));
//    diff++;
//    send++;


    gettimeofday(&prevClock,NULL);
    while(1){

        //waiting until we read a frame
        if (XbeeReadFrame(&struIn)){
            if (struIn.APID==XBEE_APID_TXS){
                statused++;
                printf("frame statused ");
                if (struIn.data.TXStatus.status==XBEE_TX_S_SUCCESS){
                    acked++;
                    diff--;
                    printf("and acked");
                }
            }
            else {
                printf(":%s:\n",struIn.data.RX16Data.payload);
                diff++;
                send++;
            }
            printf("\033[128D\033[50C| send : %d, diffAcked :%d, diffStat : %d\n",send,diff,send-statused);
        }

        gettimeofday(&currentClock,NULL);
        if ((currentClock.tv_usec-prevClock.tv_usec)>1000000){
            gettimeofday(&prevClock,NULL);
            if (XbeeTx16(0x1234,0,0x88,str,strlen(str))){
                printf("Frame TX required\n");
                diff++;
                send++;
            }
            else printf("Frame TX error\n");
        }

    }


    serialDeInit();
    return 0;
}


