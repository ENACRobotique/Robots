/*
 * main.c
 *
 *  Created on: 14 juin 2013
 *      Author: quentin
 */

#include "Xbee_API.h"
#include <stdio.h>

int main(int argc, char *argv[]){
    spAPISpecificStruct stru;
    int i=0;
    uint8_t byte;

    serialInit(0,argv[1]);

    while(testTimeout(0)) printf("test\n");

    //wait until the first frame is received (modem satus 0 : hardware reset)
    do{
        XbeeReadFrame(&stru);
    }while(stru.APID!=0x8a || stru.data.modemStatus!=XBEE_MODEM_S_HARDRST);

    while(1){
        XbeeATCmd("AP",0x42,XBEE_ATCMD_GET,0);
        printf("\ncmd send\n");

        while (!XbeeReadFrame(&stru)) printf("waiting\n");
        printf("frame received, %d\n",i);
        i++;

    }

    printf("APID : %x\n",stru.APID);
    printf("Frame ID  : %x\n",stru.data.ATResponse.frameID);
    printf("AT cmd : %c%c\n",stru.data.ATResponse.cmd[0],stru.data.ATResponse.cmd[1]);
    printf("val : 0x");
    for (i=0;i<16;i++) printf("%x",stru.data.ATResponse.value_be[i]);
    printf("\n");
    printf("fin programme \n");

    serialDeInit();
    return 0;
}


