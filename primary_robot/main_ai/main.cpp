/*
 * main.c
 *
 *  Created on: 10 oct. 2013
 *      Author: Ludo6431
 *
 * main.cpp
 *
 *  Created on: 21 dec. 2014
 *      Author: Seb
 */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <getopt.h> //already exist extern "C"

extern "C"{
#include <unistd.h> //for uslepp
}

#include "botNet_core.h"
#include "communications.h"
#include "variables.h"
#include "ai.h"

#ifdef CTRLC_MENU
static int menu = 0;

void intHandler(int dummy) {
    menu = 1;
}
#endif

void usage(char *cl) {
    printf("main ia\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--mode,     -m        AI mode (slave | auto | prog | fire)\n");
    printf("\t--log-file, -f        output log file of received messages (overwritten)\n");
    printf("\t--verbose,  -v        increases verbosity\n");
    printf("\t--quiet,    -q        not verbose\n");
    printf("\t--help,     -h, -?    prints this help\n");
}

int main(int argc, char **argv) {
    int ret;
    char verbose = 1;
    ofstream file("log.txt");
    eAIState_t eAIState = E_AI_SLAVE;

#ifdef CTRLC_MENU
    char cmd;
    bn_Address destAd;
    int quit=0,quitMenu=0;
#endif

    // arguments parsing
    while (1) {
        static struct option long_options[] = { { "mode", required_argument, NULL, 'm' }, { "log-file", required_argument, NULL, 'f' }, { "verbose", no_argument, NULL, 'v' }, { "quiet", no_argument, NULL, 'q' }, { "help", no_argument, NULL, 'h' }, { NULL, 0, NULL, 0 } };

        int c = getopt_long(argc, argv, "m:f:vqh?", long_options, NULL);
        if (c == -1)
            break;
        switch (c) {
            case 'm':
                if (!strcasecmp(optarg, "slave")) {
                    eAIState = E_AI_SLAVE;
                }
                else if (!strcasecmp(optarg, "prog")) {
                    eAIState = E_AI_PROG;
                }
                else if (!strcasecmp(optarg, "auto")) {
                    eAIState = E_AI_AUTO;
                }
                break;
            case 'f':
                file.close();
                file.open(optarg);
                break;
            case 'v':
                verbose++;
                break;
            case 'q':
                verbose = 0;
                break;

            default:
                cerr << "[ERROR] [main.cpp] ?? getopt returned character code 0" << c << "??" << endl;
                /* no break */
            case 'h':
            case '?':
                usage(argv[0]);
                exit(EXIT_FAILURE);
                break;
        }
    }

    // network initialization
    if ((ret = bn_attach(E_ROLE_SETUP, role_setup)) < 0){
        cerr << "[ERROR] [main.cpp] bn_attach() error : " << -ret << endl;
        exit(EXIT_FAILURE);
    }

    if ((ret = bn_init()) < 0) {
        cerr << "[ERROR] [main.cpp] bn_init() error : " << -ret << endl;
        exit(EXIT_FAILURE);
    }

    if ((ret = ping()) < 0){
        cerr << "[ERROR] [main.cpp] ping()" << endl;
        exit(EXIT_FAILURE);
    }

    // calls initialization functions
    switch (eAIState) {
        case E_AI_AUTO:
        case E_AI_PROG:
            ret = obj_init(eAIState);
            if (ret < 0) {
                cerr << "[ERROR] [main.cpp] obj_init() error #" << -ret << endl;
            }
            break;
        case E_AI_SLAVE:
            sPt_t pt = {INIT_POS_SLAVE_X, INIT_POS_SLAVE_Y};
            sendPos(pt, INIT_ANGLE_YELLOW);
            break;
    }

    // send all obs to monitoring if activate
    for(int i = 0 ; i < N ; i++)
        if(obs[i].active)
            obs_updated[i]++;

    cout << "[INFO] Initialization is finished" << endl;

#ifdef CTRLC_MENU
    signal(SIGINT, intHandler);
    printf("listening, CTRL+C  for menu\n");

    while (!quit){
        int nbTraces,f; //for traceroute display
#else

    // loop
    while (1){
#endif
        usleep(500);

        // check if receiving new messages
        checkInbox(verbose, file);

        // calls loop functions
        switch (eAIState) {
            case E_AI_SLAVE:
                sPt_t goal;
                if (lastGoal(goal, true)) {
                    cout << "[INFO] New goal available" << endl;
                    path.go2Point(goal, true);
                }
                break;
            case E_AI_AUTO:
            case E_AI_PROG:
                obj_step(eAIState);
                break;
            default:
                break;
        }

        // sending obstacles to monitoring
        sendObss();

        // menu
#ifdef CTRLC_MENU
        if (menu) {
            quitMenu=0;
            while (!quitMenu) {
                printf("\ndebug reader menu\n");
                printf("s : send debugger address\n");
                printf("p : ping\n");
                printf("t : traceroute\n");
                printf("i : info about this node\n");
                printf("l : add/remove line return to debug strings if missing\n");
                printf("v : increase verbosity\n");
                printf("V : decrease verbosity\n");
                printf("r : return\n");
                printf("q : quit\n");

                while(isspace(cmd=getchar()));

                switch (cmd) {
                    case 'd':
                    {
                        sMsg msg;
                        msg.header.type = E_ROLE_SETUP;
                        msg.header.destAddr = ADDRD_MAIN_PROP_SIMU;
                        msg.header.size = 2 + 4;
                        msg.payload.roleSetup.nb_steps = 1;
                        // step #0
                        msg.payload.roleSetup.steps[0].step = UPDATE_ACTIONS;
                        msg.payload.roleSetup.steps[0].type = E_DEBUG;
                        msg.payload.roleSetup.steps[0].actions.sendTo.first = ROLE_DEBUG;
                        msg.payload.roleSetup.steps[0].actions.sendTo.second = ROLE_MONITORING;
                        msg.payload.roleSetup.steps[0].actions.relayTo.n1 = 0;
                        msg.payload.roleSetup.steps[0].actions.relayTo.n2 = 0;

                        ret = bn_send(&msg);
                        if(ret < 0) {
                            printf("bn_send(E_ROLE_SETUP) error #%i\n", -ret);
                        }
                    }
                    break;
                    case 's' :  //sends debug address to distant node
                    do {
                        printf("enter destination address\n");
                        ret = scanf("%hx",&destAd);
                        if (ret != 1) {
                            printf("error getting destination address\n");
                        }
                    }while(ret != 1);
                    if ( (ret=bn_debugSendAddr(destAd)) > 0) {
                        printf("signalling send\n");
                        quitMenu=1;
                    }
                    else {
                        printf("error while sending : %d\n", ret);

                    }
                    break;
                    case 'p' :
                    do {
                        printf("enter destination address\n");
                        ret = scanf("%hx",&destAd);
                        if (ret != 1) {
                            printf("error getting destination address\n");
                        }
                    }while(ret != 1);
                    printf("ping %hx : %d ms\n",destAd,bn_ping(destAd));
                    break;
                    case 't' :
                    {
                        sTraceInfo *trInfo=NULL;
                        int depth;
                        do {
                            printf("enter destination address\n");
                            ret = scanf("%hx",&destAd);
                            if (ret != 1) {
                                printf("error getting destination address\n");
                            }
                        }while(ret != 1);
                        do {
                            printf("enter depth\n");
                            ret = scanf("%i",&depth);
                            if (ret != 1 || depth <= 0) {
                                printf("error getting depth\n");
                            }
                        }while(ret != 1 || depth <= 0);
                        trInfo = (sTraceInfo *)malloc(depth * sizeof(sTraceInfo));
                        nbTraces=bn_traceroute(destAd,trInfo,depth,1000);
                        for (f=0;f<nbTraces;f++) {
                            printf("%hx in %d ms\n",trInfo[f].addr,trInfo[f].ping);
                        }
                    }
                    break;
                    case 'i' :  //displays info about current node
#if MYADDRX
                    printf("xBee address:\n");
                    printf("  total : %4hx\n",MYADDRX);
                    printf("  local : %4hx\n",MYADDRX&DEVICEX_MASK);
                    printf("  subnet: %4hx\n",MYADDRX&SUBNET_MASK);
#endif
#if MYADDRI
                    printf("I²C address:\n");
                    printf("  total : %4hx\n",MYADDRI);
                    printf("  local : %4hx\n",MYADDRI&DEVICEI_MASK);
                    printf("  subnet: %4hx\n",MYADDRI&SUBNET_MASK);
#endif
#if MYADDRU
                    printf("UART address:\n");
                    printf("  total : %4hx\n",MYADDRU);
                    printf("  local : %4hx\n",MYADDRU&DEVICEU_MASK);
                    printf("  subnet: %4hx\n",MYADDRU&SUBNET_MASK);
#endif
#if MYADDRD
                    printf("UDP address:\n");
                    printf("  total : %4hx\n",MYADDRD);
                    printf("  local : %4hx\n",MYADDRD&DEVICED_MASK);
                    printf("  subnet: %4hx\n",MYADDRD&SUBNET_MASK);
#endif

                    printf("\n");
                    break;
                    case 'l' :
                    oLF^=1;
                    if (oLF) printf("new line will be added if missing");
                    else printf("new line won't be added if missing");
                    break;
                    case 'c' :
                    verbose^=1;
                    if (verbose) printf("message info won't be displayed");
                    else printf("message info will be displayed");
                    break;
                    case 'r' : quitMenu=1; printf("back to listening, CTRL+C for menu\n\n"); break;
                    case 'q' : quitMenu=1; quit=1; break;
                    default : break;
                }
            }

            menu=0;
        }
#endif
    }

    file.close();
    cout << "bye bye ..." << endl;

    return EXIT_SUCCESS;
}
