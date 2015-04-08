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

#include <cstdlib>
#include <cstring>
#include <getopt.h> //already exist extern "C"

extern "C"{
#include <unistd.h> //for uslepp
}

#include "botNet_core.h"
#include "communications.h"
#include "tools.h"
#include "ai.h"
#include "net.h"
#include "GeometryTools.h"

std::vector<sObs_t> initObs= {
   // robots
   {{0., 0.},          0., 1, 1, 1},   //primary
   {{0., 0.}, R_ROBOT+12., 1, 0, 1},   //secondary
   {{0., 0.}, R_ROBOT+20., 1, 0, 1},   //primary adv
   {{0., 0.}, R_ROBOT+15., 1, 0, 1},   //secondary adv

   //Yellow spots //if moved change START_STAND
   {{9.  ,180.}, 3. + R_ROBOT, 1, 1, 1}, //4
   {{85. ,180.}, 3. + R_ROBOT, 1, 1, 1},
   {{85. ,190.}, 3. + R_ROBOT, 1, 1, 1},
   {{87. ,64.5}, 3. + R_ROBOT, 1, 1, 1},
   {{9.  , 15.}, 3. + R_ROBOT, 1, 1, 1},
   {{9.  , 25.}, 3. + R_ROBOT, 1, 1, 1},
   {{110., 23.}, 3. + R_ROBOT, 1, 1, 1},
   {{130., 60.}, 3. + R_ROBOT, 1, 1, 1},

   //Green spots
   {{300 - 9.  ,180.}, 3. + R_ROBOT, 1, 1, 1}, //12
   {{300 - 85. ,180.}, 3. + R_ROBOT, 1, 1, 1},
   {{300 - 85. ,190.}, 3. + R_ROBOT, 1, 1, 1},
   {{300 - 87. ,64.5}, 3. + R_ROBOT, 1, 1, 1},
   {{300 - 9.  , 15.}, 3. + R_ROBOT, 1, 1, 1},
   {{300 - 9.  , 25.}, 3. + R_ROBOT, 1, 1, 1},
   {{300 - 110., 23.}, 3. + R_ROBOT, 1, 1, 1},
   {{300 - 130., 60.}, 3. + R_ROBOT, 1, 1, 1},

   //Cup
   {{25. , 25.}, 5. + R_ROBOT, 1, 1, 1}, //20
   {{91. ,120.}, 5. + R_ROBOT, 1, 1, 1},
   {{150., 35.}, 5. + R_ROBOT, 1, 1, 1},
   {{300 - 25. , 25.}, 5. + R_ROBOT, 1, 1, 1},
   {{300 - 91. ,120.}, 5. + R_ROBOT, 1, 1, 1},

   //Popcorn machine
   {{30. ,196.5}, 5. + R_ROBOT, 0, 1, 1}, //25
   {{60. ,196.5}, 5. + R_ROBOT, 0, 1, 1},
   {{300 - 30. ,196.5}, 5. + R_ROBOT, 0, 1, 1},
   {{300 - 60. ,196.5}, 5. + R_ROBOT, 0, 1, 1},

   //Stairs
   {{102. ,147.}, 7. + R_ROBOT, 1, 1, 1}, //29
   {{150.,200.}, 53. + R_ROBOT, 1, 1, 1},
   {{300 - 102. ,147.}, 7. + R_ROBOT, 1, 1, 1},

   //Platform
   {{125.,  5.}, 7. + R_ROBOT, 1, 1, 1}, //32
   {{300 - 125.,  5.}, 7. + R_ROBOT, 1, 1, 1},

   //Starting zone
   {{39., 79.}, 2. + R_ROBOT, 1, 1, 1}, //34
   {{20., 79.}, 2. + R_ROBOT, 1, 1, 1},
   {{39.,121.}, 2. + R_ROBOT, 1, 1, 1},
   {{20.,121.}, 2. + R_ROBOT, 1, 1, 1},
   {{300 - 39., 79.}, 2. + R_ROBOT, 1, 1, 1},
   {{300 - 20., 79.}, 2. + R_ROBOT, 1, 1, 1},
   {{300 - 39.,121.}, 2. + R_ROBOT, 1, 1, 1},
   {{300 - 20.,121.}, 2. + R_ROBOT, 1, 1, 1},

#if !HOLONOMIC
   //Cercles du robot anti-demi-tour
   {{0., 0. }, 0, 0, 0, 1}, //42
   {{0., 0. }, 0, 0, 0, 1},
   {{0., 0. }, 0, 0, 0, 1},

   //Cercles d'approches
   {{0., 0. }, 0, 0, 0, 1},//45
   {{0., 0. }, 0, 0, 0, 1},
   {{0., 0. }, 0, 0, 0, 1},
#endif

   // arrivée
   {{0. , 0.}, 0, 0, 1, 1} //48
};


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
                logs.changeFile(optarg);
                break;
            case 'v':
                verbose++;
                break;
            case 'q':
                verbose = 0;
                break;

            default:
                logs << ERR << "?? getopt returned character code 0" << c << "??";
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

    sendPing();

    // calls initialization functions
    init_obs(initObs);
    switch (eAIState) {
        case E_AI_AUTO:
        case E_AI_PROG:
            ret = initAI();
            if (ret < 0) {
                cerr << "[ERROR] [main.cpp] obj_init() error #" << -ret << endl;
            }
            break;
        case E_AI_SLAVE:
            Point2D<float> pt = {INIT_POS_SLAVE_X, INIT_POS_SLAVE_Y};
            sendPos(pt, INIT_ANGLE_SLAVE);
            break;
    }

    // send configuration of the playground and all obs to monitoring if activate
    sendObsCfg();
    for(int i = 0 ; i < N ; i++)
        if(obs[i].active)
            obs_updated[i] = 1;

    ret = 1;

    logs << INFO << "Initialization is finished";

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
        checkInbox(verbose);

        // calls loop functions
        Point2D<float> goal;
        switch (eAIState) {
            case E_AI_SLAVE:
                if (lastGoal(goal, true)) {
                    logs << INFO << "New goal available";
                    path.go2Point(goal, false);
                }
                break;
            case E_AI_AUTO:
            case E_AI_PROG:
                ret = stepAI();
                break;
            default:
                break;
        }

        // calls maintenance function
        statuses.maintenace();
        path.maintenace();
        net.maintenace();

        // sending obstacles to monitoring
        sendObss();

        // end
        if(ret == 0)
            break;

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

    logs << INFO << "bye bye ... \n";

    return EXIT_SUCCESS;
}
