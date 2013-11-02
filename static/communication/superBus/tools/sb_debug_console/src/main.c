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

#include "lib_superBus.h"
#include "lib_sbDebug.h"
#include "node_cfg.h"
#include "Xbee_API.h"
#include "Xbee4sb.h"


static int menu = 0;

void intHandler(int dummy) {
    menu = 1;
}

int main(){
    volatile int quit=0,quitMenu=0,err;
    sMsg msgIn;
    char cmd;
    sb_Address destAd;

    sb_init();

    printf("listening, CTRL+C  for menu\n");

    signal(SIGINT, intHandler);

    //main loop
    while (!quit){
        usleep(500);

        sb_routine();

        //receives messages, displays string if message is a debug message
        if (sb_receive(&msgIn)){
            printf("message received from %hx, type : %u %s\n",msgIn.header.srcAddr,msgIn.header.type,eType2str(msgIn.header.type));
            switch (msgIn.header.type){
            case E_DEBUG : printf("%s\n",msgIn.payload.debug); break;
            default : break;
            }
        }

        //menu
        if (menu){
            menu=0;
            quitMenu=0;
            while (!quitMenu){
                cmd=0;
                printf("\ndebug reader menu\n");
                printf("s : send debugger address\n");
                printf("i : info about this node\n");
                printf("r : return\n");
                printf("q : quit\n");

                while(isspace(cmd=getchar()));

                switch (toupper(cmd)){
                case 'S' :  //sends debug address to distant node
                    printf("enter destination address\n");
                    scanf("%hx",&destAd);
                    if ( (err=sb_debugSendAddr(destAd)) > 0){
                        printf("signalling send\n");
                        quitMenu=1;
                    }
                    else {
                        printf("error while sending : %d\n", err);

                    }
                    break;
                case 'I' :  //displays info about current node
                    printf("my addr (total) : %4hx\n",MYADDRX);
                    printf("my addr (local) : %4hx\n",MYADDRX&DEVICEX_MASK);
                    printf("my subnet  : %4hx\n\n",MYADDRX&SUBNET_MASK);
                    break;
                case 'R' : quitMenu=1; printf("back to listening, CTRL+C for menu\n\n"); break;
                case 'Q' : quitMenu=1; quit=1; break;
                default : break;
                }

            }

        }

    }

    printf("bye\n");

    return 0;
}
