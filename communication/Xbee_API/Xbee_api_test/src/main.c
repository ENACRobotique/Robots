/*
 * main.c
 *
 *  Created on: 14 juin 2013
 *      Author: quentin
 */

#include <stdio.h>
#include <string.h>

#include "params.h"
#include "Xbee_API.h"

int main(int argc, char *argv[]){
    spAPISpecificStruct struIn,struOut;
    int send=0,statused=0,acked=0,diff=0;
    char str[32];

    serialInit(0,argv[1]);
    printf("waiting for reset\n");
    //wait until the first frame is received (modem satus 0 : hardware reset)
    do{
        XbeeReadFrame(&struIn);
    }while(struIn.APID!=0x8a || struIn.data.modemStatus!=XBEE_MODEM_S_HARDRST);

    XbeeATCmd("MY", 42, XBEE_ATCMD_SET, MYADDRI);

        //waits for command acknowledgement
        do {
            XbeeReadFrame(&struIn);
        }while (struIn.APID!=XBEE_APID_ATRESPONSE || struIn.data.ATResponse.frameID!=42);

    sprintf(str,"ping");
    // send first ping
    XbeeTx16(0x1234,0,0x88,str,strlen(str));
    diff++;
    send++;

    while(1){
        while (XbeeReadFrame(&struIn));
        if (struIn.APID==XBEE_APID_TXS){
            statused++;
            if (struIn.data.TXStatus.status==XBEE_TX_S_SUCCESS){
                acked++;
                diff--;
            }
        }
        else {
            XbeeTx16(0x1234,0,0x88,str,strlen(str));
            diff++;
            send++;
        }

    }


    serialDeInit();
    return 0;
}


