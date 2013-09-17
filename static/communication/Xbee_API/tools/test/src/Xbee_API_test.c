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
    spAPISpecificStruct struIn;
    int send=0;

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




    while(1){

        //waiting until we read a frame
        if (XbeeReadFrame(&struIn)){
            if (struIn.APID==XBEE_APID_TXS){
            }
            else if (struIn.APID==XBEE_APID_RX16){

                printf(":%6s: %d\n",struIn.data.RX16Data.payload,send);

                XbeeTx16(0x1234,0,0x88,&send,1);
                send++;
            }

        }
        memset(&struIn,0,sizeof(struIn));

    }


    serialDeInit();
    return 0;
}


