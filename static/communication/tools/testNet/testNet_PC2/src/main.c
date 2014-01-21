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
    sMsg msgIn;

    bn_init();
    bn_attach(E_DEBUG_SIGNALLING,&bn_debugUpdateAddr);



    printf("init terminé, mon adresse est : %hx\n",MYADDRU);

    while (1){
        usleep(500);
        if (bn_routine()>0){

            printf("bn_routine >=0\n");

            if (bn_receive(&msgIn)>0){

                printf("bn_receive received\n");

            }
        }
    }

    printf("bye\n");
    bn_deattach(E_DEBUG_SIGNALLING);

    return 0;
}


